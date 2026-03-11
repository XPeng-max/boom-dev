//******************************************************************************
// See LICENSE.Berkeley for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v3.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket._

import boom.v3.common._

// ==================== Global Prefetch Monitor ====================
// Monitors prefetch accuracy and miss rate over configurable time windows (epochs).
// Can throttle individual prefetchers.
//
// Throttle logic:
//   If a prefetcher:
//      (1) send too much prefetch request (pf_issued >= 1/2 demand_issued),
//      (2) low accuracy (prefetch_useful / prefetch_fetched <= 1/4)
//   Then, We will throttle down the prefetcher in the next few epochs.
//
//   Binary exponential backoff:
//     - First throttle: 1 epoch
//     - Consecutive throttles: double duration each time (1, 2, 4, 8, ..., capped at MAX)
//     - If accuracy recovers (not throttled at epoch end), reset backoff to 1
// 
class GlobalPrefetchMonitor(numPrefetchers: Int)(implicit p: Parameters)
  extends BoomModule()(p) {
  require(numPrefetchers > 0, "GlobalPrefetchMonitor requires at least 1 prefetcher")

  val io = IO(new Bundle {
    // Per-cycle input events from dcache s2 stage
    val prefetch_issued = Input(Vec(numPrefetchers, Bool()))  // prefetch request allocated MSHR, per prefetcher
    val demand_issued   = Input(Bool())                       // demand request allocated MSHR
    val prefetch_fetched = Input(Vec(numPrefetchers, Bool())) // prefetched line filled into cache, per prefetcher
    val prefetch_useful = Input(Vec(numPrefetchers, Bool()))  // first demand hit on prefetched line, per prefetcher

    // Throttle outputs
    val pf_throttle = Output(Vec(numPrefetchers, Bool())) // per-prefetcher throttle
  })

  // === Configuration ===
  val EPOCH_BITS = 10            // 1024 cycles per epoch (~1us @ 1GHz)
  val COUNTER_BITS = 12          // Max 4095 events per epoch
  val ACCURACY_SHIFT = 2         // Low accuracy threshold: useful * 4 < fetched => accuracy < 25%
  val FREQUENCY_SHIFT = 1        // High frequency threshold: issued * 2 >= demand => issued >= 1/2 demand
  val MIN_PREFETCH = 4           // Min prefetches before evaluating (avoid noisy decisions)
  val MAX_THROTTLE_EPOCHS = 16   // Maximum backoff duration cap (in epochs)

  val MAX_BACKOFF_BITS = log2Ceil(MAX_THROTTLE_EPOCHS + 1)

  // === Epoch counter ===
  val epoch_counter = RegInit(0.U(EPOCH_BITS.W))
  val epoch_end = epoch_counter === ((1 << EPOCH_BITS) - 1).U
  epoch_counter := epoch_counter + 1.U  // wraps naturally

  val epoch_num = RegInit(0.U(16.W))
  when (epoch_end) { epoch_num := epoch_num + 1.U }

  // === Current epoch counters (saturating) ===
  val MAX_COUNT = ((1 << COUNTER_BITS) - 1).U(COUNTER_BITS.W)
  def satIncr(cnt: UInt): UInt = Mux(cnt === MAX_COUNT, MAX_COUNT, cnt + 1.U)

  val pf_issued_cnt  = RegInit(VecInit(Seq.fill(numPrefetchers)(0.U(COUNTER_BITS.W))))
  val pf_fetched_cnt = RegInit(VecInit(Seq.fill(numPrefetchers)(0.U(COUNTER_BITS.W))))
  val pf_useful_cnt  = RegInit(VecInit(Seq.fill(numPrefetchers)(0.U(COUNTER_BITS.W))))
  val demand_cnt     = RegInit(0.U(COUNTER_BITS.W))

  // Accumulate events during epoch (skip epoch_end cycle to avoid reset conflict)
  for (i <- 0 until numPrefetchers) {
    when (io.prefetch_issued(i) && !epoch_end) {
      pf_issued_cnt(i) := satIncr(pf_issued_cnt(i))
    }
    when (io.prefetch_fetched(i) && !epoch_end) {
      pf_fetched_cnt(i) := satIncr(pf_fetched_cnt(i))
    }
    when (io.prefetch_useful(i) && !epoch_end) {
      pf_useful_cnt(i) := satIncr(pf_useful_cnt(i))
    }
  }
  when (io.demand_issued && !epoch_end) {
    demand_cnt := satIncr(demand_cnt)
  }

  // === Per-prefetcher throttle state ===
  val pf_throttled = RegInit(VecInit(Seq.fill(numPrefetchers)(false.B)))
  val pf_countdown = RegInit(VecInit(Seq.fill(numPrefetchers)(0.U(MAX_BACKOFF_BITS.W))))
  val pf_backoff   = RegInit(VecInit(Seq.fill(numPrefetchers)(1.U(MAX_BACKOFF_BITS.W))))

  // === Epoch boundary evaluation ===
  when (epoch_end) {
    for (i <- 0 until numPrefetchers) {
      val issued  = pf_issued_cnt(i)
      val fetched = pf_fetched_cnt(i)
      val useful  = pf_useful_cnt(i)
      val demand  = demand_cnt

      // Condition (1): high frequency - pf_issued >= 1/2 * demand_issued
      //   <=> pf_issued * 2 >= demand_issued (avoid division)
      val high_frequency = (issued << FREQUENCY_SHIFT) >= demand

      // Condition (2): low accuracy - prefetch_useful / prefetch_fetched <= 1/4
      //   <=> prefetch_useful * 4 <= prefetch_fetched (avoid division)
      //   Only meaningful when fetched > 0
      val low_accuracy = (useful << ACCURACY_SHIFT) <= fetched

      // Need enough data to make a decision
      val enough_data = issued >= MIN_PREFETCH.U

      when (pf_throttled(i)) {
        // Currently throttled: decrement countdown
        when (pf_countdown(i) <= 1.U) {
          // Countdown expired -> unthrottle
          pf_throttled(i) := false.B
          pf_countdown(i) := 0.U
          printf(p"[PrefetchMonitor] Epoch ${epoch_num}: Prefetcher ${(i+1).U} UNTHROTTLED (backoff=${pf_backoff(i)})\n")
        } .otherwise {
          pf_countdown(i) := pf_countdown(i) - 1.U
        }
      } .otherwise {
        // Not throttled: evaluate whether to start throttling
        when (enough_data && high_frequency && low_accuracy) {
          // Both conditions met -> throttle with current backoff duration
          pf_throttled(i) := true.B
          pf_countdown(i) := pf_backoff(i)
          // Binary exponential backoff: double for next time, capped at MAX
          pf_backoff(i) := Mux(pf_backoff(i) >= MAX_THROTTLE_EPOCHS.U,
                               MAX_THROTTLE_EPOCHS.U, pf_backoff(i) << 1)
          printf(p"[PrefetchMonitor] Epoch ${epoch_num}: Prefetcher ${(i+1).U} THROTTLED " +
                 p"(issued=${issued}, fetched=${fetched}, useful=${useful}, demand=${demand}, throttle_epochs=${pf_backoff(i)})\n")
        } .elsewhen (enough_data && !high_frequency && !low_accuracy) {
          // Good behavior on both dimensions -> reset backoff
          pf_backoff(i) := 1.U
        }
      }
    }

    // Reset counters for next epoch
    for (i <- 0 until numPrefetchers) {
      pf_issued_cnt(i)  := 0.U
      pf_fetched_cnt(i) := 0.U
      pf_useful_cnt(i)  := 0.U
    }
    demand_cnt := 0.U
  }

  // === Outputs ===
  io.pf_throttle := pf_throttled
}
