//******************************************************************************
// See LICENSE.Berkeley for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v3.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._

import boom.v3.common._

trait HasAlectoParameters extends HasL1PrefetcherHelper {
    val SANDBOX_TABLE_SIZE = 256
    val SANDBOX_TABLE_BITS = log2Ceil(SANDBOX_TABLE_SIZE)
    val SANDBOX_TABLE_TAG_BITS = coreMaxAddrBits - SANDBOX_TABLE_BITS

    val SAMPLE_TABLE_SIZE = 64
    val SAMPLE_TABLE_COUNTER_WIDTH = 6
    val SAMPLE_TABLE_BITS = log2Ceil(SAMPLE_TABLE_SIZE)
    val SAMPLE_TABLE_TAG_BITS = HASH_TAG_WIDTH - SAMPLE_TABLE_BITS
    val SAMPLE_DEMAND_THRESHOLD = 50

    val ALLOCATION_TABLE_SIZE = 64
    val ALLOCATION_TABLE_BITS = log2Ceil(ALLOCATION_TABLE_SIZE)
    val ALLOCATION_TABLE_TAG_BITS = HASH_TAG_WIDTH - ALLOCATION_TABLE_BITS
    val ALLOCATION_DEGREE_WIDTH = 3

    // Epoch-based degree recovery parameters
    // When prefetch_degree is demoted to 0, record the epoch.
    // After ALLOCATION_RESET_EPOCH_THRESHOLD epochs elapse, lazily reset degree to 1 on next access.
    val ALLOCATION_EPOCH_WIDTH = 8           // 8-bit epoch counter (wraps every 256 ticks, ~131K cycles)
    val ALLOCATION_EPOCH_CYCLE_BITS = 9     // Each epoch tick = 512 cycles (~0.5us @ 1GHz)
    val ALLOCATION_RESET_EPOCH_THRESHOLD = 2 // Reset degree after 2 epoch ticks (~1024 cycles)
    val ALLOCATION_SUPPRESS_COUNT_WIDTH = 3  // 连续抑制轮次计数宽度（0~7）
    val ALLOCATION_RESET_BACKOFF_MODE = "exponential" // "linear" | "exponential"
}

// ======================= Sandbox Table =========================
class SandboxTableEntry(implicit p: Parameters) extends BoomBundle with HasAlectoParameters {
    val confirmed = Bool()
    val pc_hash = UInt(HASH_TAG_WIDTH.W) // PC标签部分
    val addr_tag = UInt(SANDBOX_TABLE_TAG_BITS.W) // 地址标签部分
    val prefetch_type = UInt(3.W) // 预取类型
}

class SandboxRequest(implicit p: Parameters) extends BoomBundle with HasAlectoParameters {
    val addr = UInt(coreMaxAddrBits.W)
    val pc = UInt(coreMaxAddrBits.W)
}

class SandboxTable(implicit p: Parameters) extends BoomModule with HasAlectoParameters {
  val io = IO(new Bundle {
    val prefetch_type = Input(UInt(3.W))
    val req = Flipped(Valid(new SandboxRequest))
    val sample_update = Valid(new SampleUpdate)
    val kill_prefetch = Output(Bool())
    val sandbox_alloc = Output(Bool()) // Indicates when a new sandbox entry is allocated (for testing/debugging)
    val sandbox_alloc_repl = Output(Bool()) // Indicates when a new sandbox entry is allocated due to replacement (for testing/debugging) 
  })

  io.kill_prefetch := false.B
  // ========== 存储结构 (SyncReadMem) ==========
  val table = SyncReadMem(SANDBOX_TABLE_SIZE, new SandboxTableEntry)
  // 独立的 valid 位数组 - SyncReadMem 复位后内容未定义，不能依赖其中的 valid 字段
  val valids = RegInit(VecInit(Seq.fill(SANDBOX_TABLE_SIZE)(false.B)))

  // ========== 地址解析辅助函数 ==========
  def getIndex(addr: UInt): UInt = addr(SANDBOX_TABLE_BITS - 1, 0)
  def getTag(addr: UInt): UInt = addr(coreMaxAddrBits - 1, SANDBOX_TABLE_BITS)

  // ========== RAW 冒险旁路寄存器 ==========
  val bypass_valid = RegInit(false.B)
  val bypass_idx = Reg(UInt(SANDBOX_TABLE_BITS.W))
  val bypass_entry = Reg(new SandboxTableEntry)

  // ========== S0: 接收请求，发起 SyncReadMem 读 ==========
  val s0_valid = io.req.valid
  val s0_idx = getIndex(io.req.bits.addr)
  val s0_pc_hash = pc_hash_tag(io.req.bits.pc)
  val s0_addr_tag = getTag(io.req.bits.addr)
  val s0_prefetch_type = io.prefetch_type
  val s0_is_prefetch = s0_prefetch_type =/= 0.U

  // 发起读请求
  val s0_read_entry = table.read(s0_idx, s0_valid)

  // ========== S1: 处理请求 ==========
  val s1_valid = RegNext(s0_valid, false.B)
  val s1_idx = RegEnable(s0_idx, s0_valid)
  val s1_pc_hash = RegEnable(s0_pc_hash, s0_valid)
  val s1_addr_tag = RegEnable(s0_addr_tag, s0_valid)
  val s1_prefetch_type = RegEnable(s0_prefetch_type, s0_valid)
  val s1_is_prefetch = RegEnable(s0_is_prefetch, s0_valid)

  // RAW 旁路处理
  val s1_bypass_hit = bypass_valid && (s1_idx === bypass_idx)
  val s1_entry = Mux(s1_bypass_hit, bypass_entry, s0_read_entry)

  // 使用独立 valids 数组判断槽位有效性（bypass 命中时一定有效，否则查 valids）
  val s1_slot_valid = s1_bypass_hit || valids(s1_idx)

  val s1_hit = s1_valid && s1_slot_valid && (s1_entry.addr_tag === s1_addr_tag)
  val s1_is_confirmed = s1_hit && (s1_entry.pc_hash === s1_pc_hash)
  val s1_need_confirmed = s1_is_confirmed && !s1_entry.confirmed


  // ========== 构造新 entry ==========
  val s1_new_entry = Wire(new SandboxTableEntry)
  s1_new_entry := s1_entry // 默认保持不变

  // ========== 默认输出 ==========
  io.sandbox_alloc := false.B
  io.sandbox_alloc_repl := false.B

  // ========== 写回逻辑 ==========
  val s1_prefetch_valid = s1_valid && s1_is_prefetch
  val s1_prefetch_alloc = s1_prefetch_valid && !s1_hit
  val s1_demand_confirmed = s1_valid && !s1_is_prefetch && s1_is_confirmed
  when (s1_prefetch_alloc) {
    // Debug: 预取请求写入sandbox
    printf(p"[SandboxTable] Prefetch recorded:s1_hit=${s1_hit}, addr_tag=0x${Hexadecimal(s1_addr_tag)}, pc_hash=0x${Hexadecimal(s1_pc_hash)}, prefetch_type=${s1_prefetch_type - 1.U}, idx=${s1_idx}\n")
    // 对于预取且未命中的请求，分配一个新的entry
    // 对于预取且命中的请求，过滤掉
    s1_new_entry.pc_hash := s1_pc_hash
    s1_new_entry.addr_tag := s1_addr_tag
    s1_new_entry.prefetch_type := s1_prefetch_type - 1.U // Alecto在Sandbox Table之后prefetch_type减1，用于进行索引
    s1_new_entry.confirmed := false.B // 初始时为false
    io.sandbox_alloc := true.B
    io.sandbox_alloc_repl := s1_slot_valid // 如果替换掉一个有效entry，则认为是repl
  } .elsewhen(s1_demand_confirmed) {
    // Debug: 需求请求查询sandbox
    // 对于需求请求，如果是命中且未确认过的，进行标记
    // 如果是命中且已确认过的，则不做任何操作
    printf(p"[SandboxTable] Demand hit CONFIRMED(need_confirmed = ${s1_need_confirmed}): addr_tag=0x${Hexadecimal(s1_addr_tag)}, pc_hash=0x${Hexadecimal(s1_pc_hash)}, prefetch_type=${s1_entry.prefetch_type}, idx=${s1_idx}\n")
    s1_new_entry.confirmed := true.B
  }
  // ========== 写回逻辑 ==========
  // 写回只在预取请求时发生（命中丢弃，未命中分配）
  // 需求请求只在confirm时写回
  val s1_should_write = s1_prefetch_alloc || s1_demand_confirmed

  when (s1_should_write) {
    table.write(s1_idx, s1_new_entry)
    valids(s1_idx) := true.B
    // 更新旁路寄存器
    bypass_valid := true.B
    bypass_idx := s1_idx
    bypass_entry := s1_new_entry
  } .otherwise {
    bypass_valid := false.B
  }

  // ========== Sample Update 输出逻辑 ==========
  val s1_prefetch_issue = s1_valid && s1_is_prefetch && !s1_hit
  val s1_demand_hit = s1_valid && !s1_is_prefetch && s1_hit
  val s1_sample_update_valid = s1_valid && (s1_prefetch_issue || s1_demand_hit)

  // 根据 s1_is_prefetch 构造 sample_update
  val s1_sample_update = Wire(new SampleUpdate)
  
  s1_sample_update.pc_hash := s1_pc_hash
  when (s1_prefetch_issue) {
    // 预取请求：输出该预取请求
    s1_sample_update.prefetch_type := s1_prefetch_type - 1.U
    s1_sample_update.is_prefetch := true.B
    s1_sample_update.is_confirmed := false.B
  } .elsewhen (s1_demand_hit) {
    // 需求请求：命中的需求请求
    s1_sample_update.prefetch_type := s1_entry.prefetch_type
    s1_sample_update.is_prefetch := false.B
    s1_sample_update.is_confirmed := s1_need_confirmed
  } .otherwise {
    s1_sample_update.prefetch_type := 0.U
    s1_sample_update.is_prefetch := false.B
    s1_sample_update.is_confirmed := false.B
  }
  when (s1_sample_update_valid) {
    printf(p"[SandboxTable] Sample Update Valid because s1_prefetch_issue = ${s1_prefetch_issue}, s1_demand_hit = ${s1_demand_hit}, SampleUpdate: pc_hash=0x${Hexadecimal(s1_sample_update.pc_hash)}, prefetch_type=${s1_sample_update.prefetch_type}, is_prefetch=${s1_sample_update.is_prefetch}, is_confirmed=${s1_sample_update.is_confirmed}\n")
  }

  io.sample_update.valid := s1_sample_update_valid
  io.sample_update.bits := s1_sample_update

  // ========== Assertions ==========
  // Validate prefetch_type is in valid range (0 = demand, 1-3 = prefetch types)
  assert(!io.req.valid || io.prefetch_type <= 4.U,
    "SandboxTable: prefetch_type out of range (must be 0-3)")
  
  // When prefetch request, prefetch_type should not be 0
  assert(!s0_valid || !s0_is_prefetch || s0_prefetch_type =/= 0.U,
    "SandboxTable: prefetch request must have non-zero prefetch_type")
  
  // Validate pc_hash is not zero for valid requests (likely indicates uninitialized)
  assert(!s1_valid || s1_pc_hash =/= 0.U,
    "SandboxTable: pc_hash is zero, may indicate uninitialized request")
}

// ======================== Sample Table =========================
class SampleTableEntry(num_prefetchers: Int)(implicit p: Parameters) extends BoomBundle with HasAlectoParameters {
    val pc_hash_tag = UInt(SAMPLE_TABLE_TAG_BITS.W) // PC标签部分
    val issued = Vec(num_prefetchers, UInt(SAMPLE_TABLE_COUNTER_WIDTH.W)) // 预取请求发送总数
    val confirmed = Vec(num_prefetchers, UInt(SAMPLE_TABLE_COUNTER_WIDTH.W)) // 匹配的预取请求总数
    val demand = UInt(SAMPLE_TABLE_COUNTER_WIDTH.W) // 需求请求总数
}

class SampleUpdate(implicit p: Parameters) extends BoomBundle with HasAlectoParameters {
    val pc_hash = UInt(HASH_TAG_WIDTH.W)
    val prefetch_type = UInt(3.W)
    val is_prefetch = Bool() // 是否是预取请求
    val is_confirmed = Bool() // 是否命中了一个需求请求
}

class AllocationUpdate(num_prefetchers: Int)(implicit p: Parameters) extends BoomBundle with HasAlectoParameters {
    val pc_hash = UInt(HASH_TAG_WIDTH.W)
    val issued = Vec(num_prefetchers, UInt(SAMPLE_TABLE_COUNTER_WIDTH.W)) // 预取请求发送总数
    val confirmed = Vec(num_prefetchers, UInt(SAMPLE_TABLE_COUNTER_WIDTH.W)) // 匹配的预取请求总数
}


class SampleTable(num_prefetchers: Int)(implicit p: Parameters) extends BoomModule with HasAlectoParameters {
  require(num_prefetchers > 0 && num_prefetchers <= 4, 
    s"SampleTable: num_prefetchers must be 1-4, got $num_prefetchers")

  val io = IO(new Bundle {
    val sample_update = Flipped(Valid(new SampleUpdate))
    val allocation_update = Decoupled(new AllocationUpdate(num_prefetchers))
    val sample_alloc = Output(Bool()) // Indicates when a new sample allocation is generated (for testing/debugging)
    val sample_alloc_repl = Output(Bool()) // Indicates when a sample allocation is generated due to replacement (for testing/debugging)
    val sample_discard_allocation_update = Output(Bool()) // Indicates when a allocation update is discarded due to pending allocation update (for testing/debugging)
    val sample_allocation_update = Output(Bool()) // Indicates when a allocation update is generated (for testing/debugging)
  })

  // ========== 存储结构 (SyncReadMem) ==========
  val table = SyncReadMem(SAMPLE_TABLE_SIZE, new SampleTableEntry(num_prefetchers))
  // 独立的 valid 位数组 - SyncReadMem 复位后内容未定义，不能依赖其中的 valid 字段
  val valids = RegInit(VecInit(Seq.fill(SAMPLE_TABLE_SIZE)(false.B)))

  // ========== 地址解析辅助函数 ==========
  def getIndex(pc_hash: UInt): UInt = pc_hash(SAMPLE_TABLE_BITS - 1, 0)
  def getTag(pc_hash: UInt): UInt = pc_hash(HASH_TAG_WIDTH - 1, SAMPLE_TABLE_BITS)

  // ========== RAW 冒险旁路寄存器 ==========
  val bypass_valid = RegInit(false.B)
  val bypass_idx = Reg(UInt(SAMPLE_TABLE_BITS.W))
  val bypass_entry = Reg(new SampleTableEntry(num_prefetchers))

  // ========== S0: 接收请求，发起 SyncReadMem 读 ==========
  val s0_valid = io.sample_update.valid
  val s0_pc_hash = io.sample_update.bits.pc_hash
  val s0_idx = getIndex(s0_pc_hash)
  val s0_tag = getTag(s0_pc_hash)
  val s0_prefetch_type = io.sample_update.bits.prefetch_type
  val s0_is_prefetch = io.sample_update.bits.is_prefetch
  val s0_is_confirmed = io.sample_update.bits.is_confirmed

  // 发起读请求
  val s0_read_entry = table.read(s0_idx, s0_valid)

  // ========== S1: 比较、更新、写回 ==========
  val s1_valid = RegNext(s0_valid, false.B)
  val s1_pc_hash = RegEnable(s0_pc_hash, s0_valid)
  val s1_idx = RegEnable(s0_idx, s0_valid)
  val s1_tag = RegEnable(s0_tag, s0_valid)
  val s1_prefetch_type = RegEnable(s0_prefetch_type, s0_valid)
  val s1_is_prefetch = RegEnable(s0_is_prefetch, s0_valid)
  val s1_is_confirmed = RegEnable(s0_is_confirmed, s0_valid)

  // RAW 旁路处理：如果上一周期刚写入同一位置，使用旁路数据
  val s1_bypass_hit = bypass_valid && (s1_idx === bypass_idx)
  val s1_read_entry = Mux(s1_bypass_hit, bypass_entry, s0_read_entry)

  // 使用独立 valids 数组判断槽位有效性
  val s1_slot_valid = s1_bypass_hit || valids(s1_idx)

  // 判断是否命中（valid 且 tag 匹配）
  val s1_hit = s1_slot_valid && (s1_read_entry.pc_hash_tag === s1_tag)

  // ========== 构造新 entry ==========
  val s1_new_entry = Wire(new SampleTableEntry(num_prefetchers))
  s1_new_entry := s1_read_entry // 默认保持不变

  // ========== 默认输出 ==========
  io.sample_alloc := false.B
  io.sample_alloc_repl := false.B

  val s1_prefetch_valid = s1_valid && s1_is_prefetch
  val s1_demand_valid = s1_valid && !s1_is_prefetch

  when (s1_prefetch_valid) {
    // 预取请求处理
    when (s1_hit) {
      // 命中：更新 issued(prefetch_type) + 1，其他字段保持
      s1_new_entry.pc_hash_tag := s1_read_entry.pc_hash_tag
      s1_new_entry.demand := s1_read_entry.demand
      for (i <- 0 until num_prefetchers) {
        s1_new_entry.confirmed(i) := s1_read_entry.confirmed(i)
        s1_new_entry.issued(i) := s1_read_entry.issued(i)
      }
      // 饱和加法 - 使用循环展开避免动态索引问题
      for (i <- 0 until num_prefetchers) {
        when (s1_prefetch_type === i.U) {
          when (s1_read_entry.issued(i) < ((1.U << SAMPLE_TABLE_COUNTER_WIDTH) - 1.U)) {
            s1_new_entry.issued(i) := s1_read_entry.issued(i) + 1.U
          }
        }
      }
    } .otherwise {
      // 未命中：替换 entry，初始化
      s1_new_entry.pc_hash_tag := s1_tag
      s1_new_entry.demand := 0.U
      // 初始化所有计数器为 0，然后设置对应类型的 issued 为 1
      for (i <- 0 until num_prefetchers) {
        s1_new_entry.issued(i) := Mux(s1_prefetch_type === i.U, 1.U, 0.U)
        s1_new_entry.confirmed(i) := 0.U
      }
      io.sample_alloc := true.B
      io.sample_alloc_repl := s1_slot_valid // 如果替换掉一个有效entry，则认为是repl
    }
    printf(p"[SampleTable] Issued updated for prefetch_type ${s1_prefetch_type}, pc_hash: ${s1_pc_hash}, s1_new_demand: ${s1_new_entry.demand}\n")
    for (i <- 0 until num_prefetchers) {
      printf(p"[SampleTable] Issued[${i}] = ${s1_new_entry.issued(i)}, Confirmed[${i}] = ${s1_new_entry.confirmed(i)}; ")
    }
    printf(p"\n")
  } .elsewhen (s1_demand_valid && s1_hit) {
    // 命中：更新 demand + 1
    s1_new_entry.pc_hash_tag := s1_read_entry.pc_hash_tag
    for (i <- 0 until num_prefetchers) {
      s1_new_entry.issued(i) := s1_read_entry.issued(i)
      s1_new_entry.confirmed(i) := s1_read_entry.confirmed(i)
    }
    // 饱和加法
    s1_new_entry.demand := Mux(
      s1_read_entry.demand < ((1.U << SAMPLE_TABLE_COUNTER_WIDTH) - 1.U),
      s1_read_entry.demand + 1.U,
      s1_read_entry.demand
    )
    when (s1_read_entry.demand === SAMPLE_DEMAND_THRESHOLD.U) {
      s1_new_entry.demand := 1.U
    }
    printf(p"[SampleTable] Demand updated for prefetch_type ${s1_prefetch_type}, pc_hash: ${s1_pc_hash}, demand: ${s1_new_entry.demand}\n")
    // 如果 is_confirmed，更新 confirmed(prefetch_type) - 使用循环展开
    when (s1_is_confirmed) {
      for (i <- 0 until num_prefetchers) {
        when (s1_prefetch_type === i.U) {
          when (s1_read_entry.confirmed(i) < ((1.U << SAMPLE_TABLE_COUNTER_WIDTH) - 1.U)) {
            s1_new_entry.confirmed(i) := s1_read_entry.confirmed(i) + 1.U
          }
        }
      }

      printf(p"[SampleTable] Confirmed updated for prefetch_type ${s1_prefetch_type}, pc_hash: ${s1_pc_hash}, s1_new_demand: ${s1_new_entry.demand}\n")
      for (i <- 0 until num_prefetchers) {
        printf(p"[SampleTable] Issued[${i}] = ${s1_new_entry.issued(i)}, Confirmed[${i}] = ${s1_new_entry.confirmed(i)}; ")
      }
      printf(p"\n")
    }
  }

  // ========== 写回逻辑 ==========
  // 只有在有效更新时才写回：
  // - 预取请求总是写回（命中更新或未命中替换）
  // - 需求请求只在命中时写回
  val s1_should_write = s1_valid && (s1_is_prefetch || s1_hit)

  when (s1_should_write) {
    table.write(s1_idx, s1_new_entry)
    valids(s1_idx) := true.B
    // 更新旁路寄存器 - 逐字段赋值避免 CIRCT packed array 问题
    bypass_valid := true.B
    bypass_idx := s1_idx
    bypass_entry.pc_hash_tag := s1_new_entry.pc_hash_tag
    bypass_entry.demand := s1_new_entry.demand
    for (i <- 0 until num_prefetchers) {
      bypass_entry.issued(i) := s1_new_entry.issued(i)
      bypass_entry.confirmed(i) := s1_new_entry.confirmed(i)
    }
  } .otherwise {
    bypass_valid := false.B
  }

  // ========== Allocation 输出逻辑 ==========
  // 当 demand >= SAMPLE_DEMAND_THRESHOLD 时，输出 allocation_update
  // 使用更新后的 demand 值进行判断
  val s1_demand_threshold_reached = s1_valid && s1_hit && !s1_is_prefetch && 
                                     (s1_new_entry.demand >= SAMPLE_DEMAND_THRESHOLD.U)

  // TODO: 记录丢失的更新数量
  // 使用寄存器保存 allocation 输出，直到被接收
  val alloc_valid = RegInit(false.B)
  val alloc_bits = Reg(new AllocationUpdate(num_prefetchers))

  io.sample_discard_allocation_update := alloc_valid && !io.allocation_update.fire && s1_demand_threshold_reached // 如果当前有未被接收的 allocation 更新，则记录丢弃
  io.sample_allocation_update := s1_demand_threshold_reached // 记录allocation更新请求数量
  when (s1_demand_threshold_reached && (!alloc_valid || io.allocation_update.fire)) {
    // 触发新的 allocation 输出
    alloc_valid := true.B
    alloc_bits.pc_hash := s1_pc_hash
    // 逐元素赋值避免 CIRCT packed array 问题
    for (i <- 0 until num_prefetchers) {
      alloc_bits.issued(i) := s1_new_entry.issued(i)
      alloc_bits.confirmed(i) := s1_new_entry.confirmed(i)
    }
  } .elsewhen (io.allocation_update.fire) {
    alloc_valid := false.B
  }

  io.allocation_update.valid := alloc_valid
  io.allocation_update.bits := alloc_bits

  // ========== Assertions ==========
  // Validate prefetch_type is within range for indexing
  assert(!s1_valid || !s1_is_prefetch || s1_prefetch_type < num_prefetchers.U,
    "SampleTable: prefetch_type exceeds num_prefetchers")
  
  // Validate confirmed <= issued for all prefetchers (sanity check)
  for (i <- 0 until num_prefetchers) {
    assert(!s1_valid || !s1_hit || s1_read_entry.confirmed(i) <= s1_read_entry.issued(i),
      s"SampleTable: confirmed($i) should not exceed issued($i)")
  }
  
  // Ensure we don't lose allocation updates due to backpressure for too long
  val alloc_pending_cycles = RegInit(0.U(8.W))
  when (alloc_valid && !io.allocation_update.fire) {
    alloc_pending_cycles := alloc_pending_cycles + 1.U
  } .otherwise {
    alloc_pending_cycles := 0.U
  }
  assert(alloc_pending_cycles < 255.U,
    "SampleTable: allocation_update pending for too long, possible deadlock")
}
// ======================== Allocation Table =========================

class AllocationTableEntry(num_prefetchers: Int)(implicit p: Parameters) extends BoomBundle with HasAlectoParameters {
    val pc_hash_tag = UInt(SAMPLE_TABLE_TAG_BITS.W) // PC标签部分
    val prefetch_degree = Vec(num_prefetchers, UInt(ALLOCATION_DEGREE_WIDTH.W)) // 对于各个预取器的预取度
    val demote_epoch = Vec(num_prefetchers, UInt(ALLOCATION_EPOCH_WIDTH.W)) // 记录 degree 降为 0 时的 epoch
    val suppress_count = Vec(num_prefetchers, UInt(ALLOCATION_SUPPRESS_COUNT_WIDTH.W)) // 连续被抑制轮次
    val recovered = Vec(num_prefetchers, Bool()) // 标记当前 degree 是否来自 epoch 恢复
}

class AllocationRequest(implicit p: Parameters) extends BoomBundle with HasAlectoParameters {
    val pc = UInt(coreMaxAddrBits.W)
}

class AllocationResponseBundle(num_prefetchers: Int)(implicit p: Parameters) extends BoomBundle with HasAlectoParameters {
    val prefetch_degree = Vec(num_prefetchers, UInt(ALLOCATION_DEGREE_WIDTH.W))
}


class AllocationTable(num_prefetchers: Int)(implicit p: Parameters) extends BoomModule with HasAlectoParameters {
  require(num_prefetchers > 0 && num_prefetchers <= 4, 
    s"AllocationTable: num_prefetchers must be 1-4, got $num_prefetchers")
  require(ALLOCATION_RESET_BACKOFF_MODE == "linear" || ALLOCATION_RESET_BACKOFF_MODE == "exponential",
    s"AllocationTable: ALLOCATION_RESET_BACKOFF_MODE must be 'linear' or 'exponential', got $ALLOCATION_RESET_BACKOFF_MODE")

  val io = IO(new Bundle {
    val allocation_update = Flipped(Decoupled(new AllocationUpdate(num_prefetchers)))
    val allocation_req = Flipped(Valid(new AllocationRequest))
    val allocation_resp = Valid(new AllocationResponseBundle(num_prefetchers))
    val allocation_alloc = Output(Bool()) // Indicates when a new allocation entry is generated (for testing/debugging)
    val allocation_alloc_repl = Output(Bool()) // Indicates when a new allocation entry is generated due to replacement (for testing/debugging) 
  })
  val table = SyncReadMem(ALLOCATION_TABLE_SIZE, new AllocationTableEntry(num_prefetchers))
  // 独立的 valid 位数组 - SyncReadMem 复位后内容未定义，不能依赖其中的 valid 字段
  val valids = RegInit(VecInit(Seq.fill(ALLOCATION_TABLE_SIZE)(false.B)))

  // ========== Global epoch counter for degree recovery ==========
  // When a prefetcher's degree is demoted to 0, we record the current epoch.
  // On subsequent accesses, if enough epochs have passed, we lazily reset the degree to 1,
  // giving the prefetcher another chance.
  val epoch_counter = RegInit(0.U(ALLOCATION_EPOCH_CYCLE_BITS.W))
  val global_epoch = RegInit(0.U(ALLOCATION_EPOCH_WIDTH.W))
  val epoch_tick = epoch_counter === ((1 << ALLOCATION_EPOCH_CYCLE_BITS) - 1).U
  epoch_counter := epoch_counter + 1.U
  when (epoch_tick) {
    global_epoch := global_epoch + 1.U
  }

  // Helper: adaptive threshold based on continuous suppression count
  // linear:      threshold = base + suppress_count
  // exponential: threshold = base * (2 ^ suppress_count)
  // both modes saturate to epoch-width max.
  def adaptiveResetThreshold(suppress_count: UInt): UInt = {
    val max_threshold = (BigInt(1) << ALLOCATION_EPOCH_WIDTH) - 1
    val table = VecInit((0 until (1 << ALLOCATION_SUPPRESS_COUNT_WIDTH)).map { i =>
      val value = if (ALLOCATION_RESET_BACKOFF_MODE == "linear") {
        BigInt(ALLOCATION_RESET_EPOCH_THRESHOLD) + i
      } else {
        BigInt(ALLOCATION_RESET_EPOCH_THRESHOLD) << i
      }
      val saturated = if (value > max_threshold) max_threshold else value
      saturated.U(ALLOCATION_EPOCH_WIDTH.W)
    })
    table(suppress_count)
  }

  // Helper: check if a degree-0 entry has cooled down enough to be reset to 1
  def epochExpired(degree: UInt, demote_epoch: UInt, suppress_count: UInt): Bool = {
    degree === 0.U && ((global_epoch - demote_epoch) >= adaptiveResetThreshold(suppress_count))
  }

  // ========== 地址解析辅助函数 ==========
  def getIndex(pc_hash: UInt): UInt = pc_hash(ALLOCATION_TABLE_BITS - 1, 0)
  def getTag(pc_hash: UInt): UInt = pc_hash(HASH_TAG_WIDTH - 1, ALLOCATION_TABLE_BITS)

  val s0_pc_hash = pc_hash_tag(io.allocation_req.bits.pc)

  val s0_req_idx = getIndex(s0_pc_hash)
  val s0_req_tag = getTag(s0_pc_hash)

  // ========== RAW 冒险旁路寄存器 ==========
  val bypass_valid = RegInit(false.B)
  val bypass_idx = Reg(UInt(ALLOCATION_TABLE_BITS.W))
  val bypass_entry = Reg(new AllocationTableEntry(num_prefetchers))

  // ========== 读端口优先级：allocation_req > allocation_update ==========
  val has_req = io.allocation_req.valid
  val has_update = io.allocation_update.valid && !has_req

  // ========== 读请求 ==========
  val read_idx = Mux(has_req, s0_req_idx, getIndex(io.allocation_update.bits.pc_hash))
  val read_entry = table.read(read_idx, has_req || has_update)

  // ========== S1: allocation_req 处理 ==========
  val s1_req_valid = RegNext(has_req, false.B)
  val s1_req_pc_hash = RegEnable(s0_pc_hash, has_req)
  val s1_req_idx = RegEnable(s0_req_idx, has_req)
  val s1_req_tag = RegEnable(s0_req_tag, has_req)

  // RAW 旁路处理
  val s1_req_bypass_hit = bypass_valid && (s1_req_idx === bypass_idx)
  val s1_req_read_entry = Mux(s1_req_bypass_hit, bypass_entry, read_entry)

  // 使用独立 valids 数组判断槽位有效性
  val s1_req_slot_valid = s1_req_bypass_hit || valids(s1_req_idx)

  // 判断是否命中
  val s1_req_hit = s1_req_slot_valid && (s1_req_read_entry.pc_hash_tag === s1_req_tag)

  // 构造响应 - 包含 epoch-based degree recovery
  val s1_req_resp = Wire(new AllocationResponseBundle(num_prefetchers))
  when (s1_req_hit) {
    for (i <- 0 until num_prefetchers) {
      // 如果 degree==0 且 epoch 已过期，返回 1（给预取器重新尝试的机会）
      s1_req_resp.prefetch_degree(i) := Mux(
        epochExpired(
          s1_req_read_entry.prefetch_degree(i),
          s1_req_read_entry.demote_epoch(i),
          s1_req_read_entry.suppress_count(i)),
        1.U,
        s1_req_read_entry.prefetch_degree(i)
      )
    }
  } .otherwise {
    // 未命中，返回默认值 1.U
    for (i <- 0 until num_prefetchers) {
      s1_req_resp.prefetch_degree(i) := 1.U
    }
  }

  io.allocation_resp.valid := s1_req_valid
  io.allocation_resp.bits := s1_req_resp

  // Debug: allocation_req 查询结果
  when (s1_req_valid) {
    when (s1_req_hit) {
      printf(p"[AllocationTable] Req HIT: pc_hash=0x${Hexadecimal(s1_req_pc_hash)}, idx=${s1_req_idx}, degrees=")
      for (i <- 0 until num_prefetchers) {
        val raw_deg = s1_req_read_entry.prefetch_degree(i)
        val expired = epochExpired(raw_deg, s1_req_read_entry.demote_epoch(i), s1_req_read_entry.suppress_count(i))
        printf(p"[${i.U}]=${s1_req_resp.prefetch_degree(i)}")
        when (expired) { printf(p"(reset)") }
        printf(p" ")
      }
      printf(p"\n")
    } .otherwise {
      printf(p"[AllocationTable] Req MISS: pc_hash=0x${Hexadecimal(s1_req_pc_hash)}, idx=${s1_req_idx}, returning default degrees=1\n")
    }
  }

  // ========== S1: allocation_update 处理 ==========
  val s1_update_valid = RegNext(has_update, false.B)
  val s1_update_pc_hash = RegEnable(io.allocation_update.bits.pc_hash, has_update)
  val s1_update_idx = RegEnable(getIndex(io.allocation_update.bits.pc_hash), has_update)
  val s1_update_tag = RegEnable(getTag(io.allocation_update.bits.pc_hash), has_update)
  val s1_update_issued = RegEnable(io.allocation_update.bits.issued, has_update)
  val s1_update_confirmed = RegEnable(io.allocation_update.bits.confirmed, has_update)

  // RAW 旁路处理
  val s1_update_bypass_hit = bypass_valid && (s1_update_idx === bypass_idx)
  val s1_update_read_entry = Mux(s1_update_bypass_hit, bypass_entry, read_entry)

  // 使用独立 valids 数组判断槽位有效性
  val s1_update_slot_valid = s1_update_bypass_hit || valids(s1_update_idx)

  // 判断是否命中
  val s1_update_hit = s1_update_slot_valid && (s1_update_read_entry.pc_hash_tag === s1_update_tag)

  // 计算 is_promote 和 is_demote
  val is_promote = VecInit((0 until num_prefetchers).map { i =>
    (s1_update_confirmed(i) >= ((s1_update_issued(i) >> 1) + (s1_update_issued(i) >> 2))) && (s1_update_issued(i) =/= 0.U)
  })
  val is_demote = VecInit((0 until num_prefetchers).map { i =>
    (s1_update_confirmed(i) <= (s1_update_issued(i) >> 2)) && (s1_update_issued(i) =/= 0.U)
  })

  // 构造新 entry
  val s1_new_entry = Wire(new AllocationTableEntry(num_prefetchers))
  s1_new_entry.pc_hash_tag := Mux(s1_update_hit, s1_update_read_entry.pc_hash_tag, s1_update_tag)
  
  val max_degree = (1.U << ALLOCATION_DEGREE_WIDTH) - 1.U
  val max_suppress_count = (1.U << ALLOCATION_SUPPRESS_COUNT_WIDTH) - 1.U
  for (i <- 0 until num_prefetchers) {
    // base_degree: 命中时从旧 entry 读取，同时应用 epoch recovery
    val raw_degree = Mux(s1_update_hit, s1_update_read_entry.prefetch_degree(i), 1.U)
    // 更新时命中了一个表项，且该表项相对于上一次更新已经过了足够的 epoch，认为是“冷”了，重置 degree 到 1
    val epoch_reset = s1_update_hit && epochExpired(
      s1_update_read_entry.prefetch_degree(i),
      s1_update_read_entry.demote_epoch(i),
      s1_update_read_entry.suppress_count(i))
    val base_degree = Mux(epoch_reset, 1.U, raw_degree)
    
    s1_new_entry.prefetch_degree(i) := MuxCase(base_degree, Seq(
      is_promote(i) -> Mux(base_degree < max_degree, base_degree + 1.U, base_degree),
      is_demote(i)  -> Mux(base_degree > 0.U, base_degree - 1.U, base_degree)
    ))

    // ---- 恢复检测与抑制轮次逻辑 ----
    // old_was_zero: SRAM中degree本身就是0且尚未epoch恢复（仍在冷却期）
    val old_was_zero = s1_update_hit && (s1_update_read_entry.prefetch_degree(i) === 0.U) && !epoch_reset
    val old_suppress_count = Mux(s1_update_hit, s1_update_read_entry.suppress_count(i), 0.U)
    val old_recovered = s1_update_hit && s1_update_read_entry.recovered(i)
    val increased_suppress_count = Mux(old_suppress_count < max_suppress_count,
      old_suppress_count + 1.U,
      old_suppress_count)

    // 判断"恢复后再次被demote到0"的两种情况：
    // 1) 同一update内: epoch_reset=true → base_degree=1 → demote → new_degree=0
    // 2) 跨update: 上一次update写回了degree>0+recovered=true，本次update demote到0
    val transition_to_zero = (s1_new_entry.prefetch_degree(i) === 0.U) && !old_was_zero
    val re_suppress_after_recovery = transition_to_zero && (epoch_reset || old_recovered)

    // demote_epoch:
    //   - degree==0 且一直是0 (old_was_zero): 保留旧 epoch
    //   - degree==0 且刚降到0 (transition_to_zero): 记录当前 epoch
    //   - degree>0: 无意义，置0
    s1_new_entry.demote_epoch(i) := Mux(s1_new_entry.prefetch_degree(i) === 0.U,
      Mux(old_was_zero, s1_update_read_entry.demote_epoch(i), global_epoch),
      0.U
    )

    // suppress_count:
    //   - degree==0 且 old_was_zero: 保留旧值
    //   - degree==0 且 re_suppress: 自增
    //   - degree==0 且首次demote(非恢复后): 归零
    //   - degree>0: 归零
    s1_new_entry.suppress_count(i) := Mux(s1_new_entry.prefetch_degree(i) === 0.U,
      Mux(old_was_zero,
        old_suppress_count,
        Mux(re_suppress_after_recovery, increased_suppress_count, 0.U)),
      0.U
    )

    // recovered 标记:
    //   - epoch恢复后 degree>0: 设为true（标记来源是恢复）
    //   - degree被promote到>1（真正站稳了）: 清除 recovered
    //   - degree==0: 无意义，清除
    //   - 其他(degree==1但非恢复来源): 保留旧值
    s1_new_entry.recovered(i) := Mux(s1_new_entry.prefetch_degree(i) === 0.U,
      false.B,
      Mux(s1_new_entry.prefetch_degree(i) > 1.U,
        false.B, // degree>1 说明已经成功promote，不再视为"刚恢复"
        Mux(epoch_reset, true.B, old_recovered) // degree==1: 若本次恢复则标记, 否则保留旧值
      )
    )
  }

  // ========== 默认输出 ==========
  io.allocation_alloc := false.B
  io.allocation_alloc_repl := false.B

  // ========== 写回逻辑 ==========
  val s1_should_write = s1_update_valid

  when (s1_should_write) {
    io.allocation_alloc := !s1_update_hit
    io.allocation_alloc_repl := !s1_update_hit && s1_update_slot_valid
    table.write(s1_update_idx, s1_new_entry)
    valids(s1_update_idx) := true.B
    // 更新旁路寄存器 - 逐字段赋值避免 CIRCT packed array 问题
    bypass_valid := true.B
    bypass_idx := s1_update_idx
    bypass_entry.pc_hash_tag := s1_new_entry.pc_hash_tag
    for (i <- 0 until num_prefetchers) {
      bypass_entry.prefetch_degree(i) := s1_new_entry.prefetch_degree(i)
      bypass_entry.demote_epoch(i) := s1_new_entry.demote_epoch(i)
      bypass_entry.suppress_count(i) := s1_new_entry.suppress_count(i)
      bypass_entry.recovered(i) := s1_new_entry.recovered(i)
    }
    
    // Debug: allocation_update 更新结果
    printf(p"[AllocationTable] Update: pc_hash=0x${Hexadecimal(s1_update_pc_hash)}, idx=${s1_update_idx}, hit=${s1_update_hit}, epoch=${global_epoch}\n")
    for (i <- 0 until num_prefetchers) {
      val raw_degree = Mux(s1_update_hit, s1_update_read_entry.prefetch_degree(i), 1.U)
      val epoch_reset = s1_update_hit && epochExpired(
        s1_update_read_entry.prefetch_degree(i),
        s1_update_read_entry.demote_epoch(i),
        s1_update_read_entry.suppress_count(i))
      val old_degree = Mux(epoch_reset, 1.U, raw_degree)
      printf(p"  [prefetcher ${i.U}] issued=${s1_update_issued(i)}, confirmed=${s1_update_confirmed(i)}, ")
      printf(p"promote=${is_promote(i)}, demote=${is_demote(i)}, ")
      printf(p"degree: ${old_degree} -> ${s1_new_entry.prefetch_degree(i)}")
      when (epoch_reset) { printf(p" (epoch_reset)") }
      when (s1_new_entry.prefetch_degree(i) === 0.U) {
        printf(p" demote_epoch=${s1_new_entry.demote_epoch(i)} suppress_count=${s1_new_entry.suppress_count(i)}")
      }
      when (s1_new_entry.recovered(i)) { printf(p" recovered") }
      printf(p"\n")
    }
  } .otherwise {
    bypass_valid := false.B
  }

  // ========== allocation_update 握手信号 ==========
  io.allocation_update.ready := !has_req

  // ========== Assertions ==========
  // Validate prefetch_degree is within valid range
  for (i <- 0 until num_prefetchers) {
    assert(!s1_req_valid || !s1_req_hit || 
           s1_req_read_entry.prefetch_degree(i) <= ((1.U << ALLOCATION_DEGREE_WIDTH) - 1.U),
      s"AllocationTable: prefetch_degree($i) out of range")
  }
  
  // Ensure confirmed <= issued in update data (sanity check from SampleTable)
  for (i <- 0 until num_prefetchers) {
    assert(!s1_update_valid || s1_update_confirmed(i) <= s1_update_issued(i),
      s"AllocationTable: update confirmed($i) exceeds issued($i)")
  }
  
  // Warn if both req and update target same index (may cause stale data issues)
  val s0_update_idx = getIndex(io.allocation_update.bits.pc_hash)
  assert(!(has_req && has_update && s0_req_idx === s0_update_idx),
    "AllocationTable: req and update targeting same index simultaneously - potential RAW hazard")
}