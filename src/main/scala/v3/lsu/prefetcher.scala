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
import boom.v3.exu.BrResolutionInfo
import boom.v3.util.{IsKilledByBranch, GetNewBrMask, BranchKillableQueue, IsOlder, UpdateBrMask}



abstract class DataPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends BoomModule()(p)
{
  val io = IO(new Bundle {
    val mshr_avail = Input(Bool())
    val req_val    = Input(Bool())
    val req_addr   = Input(UInt(coreMaxAddrBits.W))
    val req_vaddr  = Input(UInt(coreMaxAddrBits.W))
    val req_coh    = Input(new ClientMetadata)
    val req_pc     = Input(UInt(coreMaxAddrBits.W))

    val prefetch   = Decoupled(new BoomDCacheReq)

    // TLB翻译接口
    val prefetch_translation_req = new DecoupledIO(new BoomDCacheTranslationReq)
    val prefetch_translation_resp = Flipped(new DecoupledIO(new BoomDCacheTranslationResp))
  })
}

/**
  * Does not prefetch
  */
class NullPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher
{
  io.prefetch.valid := false.B
  io.prefetch.bits  := DontCare

  // 默认不需要进行地址翻译
  io.prefetch_translation_req.valid := false.B
  io.prefetch_translation_req.bits.translation_vaddr := DontCare
}

/**
  * Next line prefetcher. Grabs the next line on a cache miss
  */
class NLPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher
{
  val req_valid = RegInit(false.B)
  val req_addr  = Reg(UInt(coreMaxAddrBits.W))
  val req_cmd   = Reg(UInt(M_SZ.W))

  val mshr_req_addr = io.req_addr + cacheBlockBytes.U
  val cacheable = edge.manager.supportsAcquireBSafe(mshr_req_addr, lgCacheBlockBytes.U)
  when (io.req_val && cacheable) {
    req_valid := true.B
    req_addr  := mshr_req_addr
    req_cmd   := Mux(ClientStates.hasWritePermission(io.req_coh.state), M_PFW, M_PFR)
  } .elsewhen (io.prefetch.fire) {
    req_valid := false.B
  }

  io.prefetch.valid            := req_valid && io.mshr_avail
  io.prefetch.bits             := DontCare
  io.prefetch.bits.addr        := req_addr
  io.prefetch.bits.uop         := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := req_cmd
  io.prefetch.bits.data        := DontCare
  io.prefetch_translation_req.valid := false.B
  io.prefetch_translation_req.bits.translation_vaddr := DontCare
  io.prefetch_translation_resp.ready := false.B
}


/**
  * Virtual Address based Next Line Prefetcher
  * Uses virtual addresses for prefetching and requires TLB translation
  */
class VAddrNLPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher
{
  val req_valid = RegInit(false.B)
  val req_vaddr = Reg(UInt(coreMaxAddrBits.W))
  val req_paddr = Reg(UInt(coreMaxAddrBits.W))
  val req_cmd   = Reg(UInt(M_SZ.W))
  val needs_translation = RegInit(false.B)
  
  // 计算下一行的虚拟地址
  val mshr_req_vaddr = io.req_vaddr + cacheBlockBytes.U
  val cacheable = edge.manager.supportsAcquireBSafe(mshr_req_vaddr, lgCacheBlockBytes.U)

  // 当收到有效请求时，准备预取下一行
  when (io.req_val && cacheable) {
    req_valid := true.B
    req_vaddr := mshr_req_vaddr
    req_cmd   := Mux(ClientStates.hasWritePermission(io.req_coh.state), M_PFW, M_PFR)
    needs_translation := true.B
  } .elsewhen (io.prefetch.fire) {
    req_valid := false.B
  }
  
  io.prefetch_translation_resp.ready := needs_translation
  // 处理翻译结果
  when (io.prefetch_translation_resp.valid && !io.prefetch_translation_resp.bits.translation_miss) {
    req_paddr := io.prefetch_translation_resp.bits.translation_paddr
    needs_translation := false.B
  } .elsewhen (io.prefetch_translation_resp.valid && io.prefetch_translation_resp.bits.translation_miss) {
    // 翻译失败，取消预取
    req_valid := false.B
    needs_translation := false.B
  }

  // 请求地址翻译
  io.prefetch_translation_req.valid := req_valid && needs_translation
  io.prefetch_translation_req.bits.translation_vaddr := req_vaddr

  // 预取请求的输出
  io.prefetch.valid            := req_valid && io.mshr_avail && !needs_translation
  io.prefetch.bits.addr        := req_paddr  // 使用翻译后的物理地址
  io.prefetch.bits.uop         := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := req_cmd
  io.prefetch.bits.data        := DontCare
  io.prefetch.bits.vaddr       := req_vaddr  // 保留原始虚拟地址
  io.prefetch.bits.is_hella    := false.B
}

// =============================================================================
// Stride Prefetcher (Not implemented yet)
// =============================================================================

trait HasL1PrefetcherHelper extends HasL1HellaCacheParameters {
  // region related
  val REGION_SIZE = 1024
  val PAGE_OFFSET = 12
  val BIT_VEC_WITDH = REGION_SIZE / cacheBlockBytes
  val REGION_BITS = log2Up(BIT_VEC_WITDH)
  val REGION_TAG_OFFSET = lgCacheBlockBytes + REGION_BITS
  val REGION_TAG_BITS = vaddrBits - lgCacheBlockBytes - REGION_BITS

  // hash related
  val VADDR_HASH_WIDTH = 5
  val BLK_ADDR_RAW_WIDTH = 10
  val HASH_TAG_WIDTH = VADDR_HASH_WIDTH + BLK_ADDR_RAW_WIDTH

  // capacity related
  val MLP_SIZE = 32
  val MLP_L1_SIZE = 16
  val MLP_L2L3_SIZE = MLP_SIZE - MLP_L1_SIZE

  // prefetch sink related
  val SINK_BITS = 2
  def SINK_L1 = "b00".U
  def SINK_L2 = "b01".U
  def SINK_L3 = "b10".U

  // vaddr: |       region tag        |  region bits  | block offset |
  def get_region_tag(vaddr: UInt) = {
    require(vaddr.getWidth == vaddrBits)
    vaddr(vaddr.getWidth - 1, REGION_TAG_OFFSET)
  }

  def get_region_bits(vaddr: UInt) = {
    require(vaddr.getWidth == vaddrBits)
    vaddr(REGION_TAG_OFFSET - 1, lgCacheBlockBytes)
  }

  def block_addr(x: UInt): UInt = {
    x(x.getWidth - 1, lgCacheBlockBytes)
  }

  def vaddr_hash(x: UInt): UInt = {
    val width = VADDR_HASH_WIDTH
    val low = x(width - 1, 0)
    val mid = x(2 * width - 1, width)
    val high = x(3 * width - 1, 2 * width)
    low ^ mid ^ high
  }

  def pc_hash_tag(x: UInt): UInt = {
    val low = x(BLK_ADDR_RAW_WIDTH - 1, 0)
    val high = x(BLK_ADDR_RAW_WIDTH - 1 + 3 * VADDR_HASH_WIDTH, BLK_ADDR_RAW_WIDTH)
    val high_hash = vaddr_hash(high)
    Cat(high_hash, low)
  }

  def block_hash_tag(x: UInt): UInt = {
    val blk_addr = block_addr(x)
    val low = blk_addr(BLK_ADDR_RAW_WIDTH - 1, 0)
    val high = blk_addr(BLK_ADDR_RAW_WIDTH - 1 + 3 * VADDR_HASH_WIDTH, BLK_ADDR_RAW_WIDTH)
    val high_hash = vaddr_hash(high)
    Cat(high_hash, low)
  }

  def region_hash_tag(region_tag: UInt): UInt = {
    val low = region_tag(BLK_ADDR_RAW_WIDTH - 1, 0)
    val high = region_tag(BLK_ADDR_RAW_WIDTH - 1 + 3 * VADDR_HASH_WIDTH, BLK_ADDR_RAW_WIDTH)
    val high_hash = vaddr_hash(high)
    Cat(high_hash, low)
  }

  def region_to_block_addr(region_tag: UInt, region_bits: UInt): UInt = {
    Cat(region_tag, region_bits)
  }

  def get_candidate_oh(x: UInt): UInt = {
    require(x.getWidth == paddrBits)
    UIntToOH(x(REGION_BITS + lgCacheBlockBytes - 1, lgCacheBlockBytes))
  }

  def toBinary(n: Int): String = n match {
    case 0|1 => s"$n"
    case _   => s"${toBinary(n/2)}${n%2}"
  }
}

trait HasStridePrefetcherConstants extends HasL1PrefetcherHelper{
  // stride table related
  val STRIDE_FILTER_SIZE = 6
  val STRIDE_ENTRY_NUM = 10
  val STRIDE_BITS = 10 + lgCacheBlockBytes
  val STRIDE_VADDR_BITS = 10 + lgCacheBlockBytes
  val STRIDE_CONF_BITS = 2

  val STRIDE_DEPTH_RATIO = 1 // prefetch depth = stride << STRIDE_DEPTH_RATIO

  // detail control
  val ALWAYS_UPDATE_PRE_VADDR = true
  // NOTE: for now, not support yet.
  val AGGRESIVE_POLICY = false // if true, prefetch degree is greater than 1, 1 otherwise
  val STRIDE_LOOK_AHEAD_BLOCKS = 2 // aggressive degree
  val LOOK_UP_STREAM = false // if true, avoid collision with stream

  // NOTE: for now, not support yet.
  val STRIDE_WIDTH_BLOCKS = if(AGGRESIVE_POLICY) STRIDE_LOOK_AHEAD_BLOCKS else 1

  def MAX_CONF = (1 << STRIDE_CONF_BITS) - 1

  // def block_addr(addr: UInt): UInt = {
  //   addr >> log2Ceil(cacheBlockBytes).U
  // }
}

class StrideMetaBundle(implicit p: Parameters) extends BoomBundle with HasStridePrefetcherConstants
{
  val pre_vaddr = UInt(STRIDE_VADDR_BITS.W)
  val stride = UInt(STRIDE_BITS.W)
  val confidence = UInt(STRIDE_CONF_BITS.W)
  val hash_pc = UInt(HASH_TAG_WIDTH.W)

  def reset(index: Int) = {
    pre_vaddr := 0.U
    stride := 0.U
    confidence := 0.U
    hash_pc := index.U
  }

  def tag_match(valid1: Bool, valid2: Bool, new_hash_pc: UInt): Bool = {
    valid1 && valid2 && hash_pc === new_hash_pc
  }

  def alloc(vaddr: UInt, alloc_hash_pc: UInt) = {
    pre_vaddr := vaddr(STRIDE_VADDR_BITS - 1, 0)
    stride := 0.U
    confidence := 0.U
    hash_pc := alloc_hash_pc
  }

  def update(vaddr: UInt, always_update_pre_vaddr: Bool) = {
    val new_vaddr = vaddr(STRIDE_VADDR_BITS - 1, 0)
    val new_stride = new_vaddr - pre_vaddr
    val new_stride_blk = block_addr(new_stride)
    // NOTE: for now, disable negtive stride
    val stride_valid = new_stride_blk =/= 0.U && new_stride(STRIDE_VADDR_BITS - 1) === 0.U
    val stride_match = new_stride === stride
    val low_confidence = confidence <= 1.U
    val can_send_pf = stride_valid && stride_match && confidence === MAX_CONF.U
    printf("StridePrefetcher: stride mis-match, new_vaddr = %x, pre_vaddr = %x, old_stride = %d, new_stride = %d, new_stride_blk = %d, stride_match = %d, low_confidence = %d\n", new_vaddr, pre_vaddr,stride, new_stride, new_stride_blk,stride_match,low_confidence)

    when(stride_valid) {
      when(stride_match) {
        confidence := Mux(confidence === MAX_CONF.U, confidence, confidence + 1.U)
      }.otherwise {
        confidence := Mux(confidence === 0.U, confidence, confidence - 1.U)
        when(low_confidence) {
          stride := new_stride
        }
      }
      pre_vaddr := new_vaddr
    }
    when(always_update_pre_vaddr) {
      pre_vaddr := new_vaddr
    }

    (can_send_pf, new_stride)
  }
}


class StridePrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher with HasStridePrefetcherConstants
{
  val array = Reg(Vec(STRIDE_ENTRY_NUM, new StrideMetaBundle))
  val valids = RegInit(VecInit(Seq.fill(STRIDE_ENTRY_NUM)(false.B)))

  def reset_array(i: Int): Unit = {
    valids(i) := false.B
    //only need to rest control signals for firendly area
    // array(i).reset(i)
  }

  val replacement = ReplacementPolicy.fromString("plru", STRIDE_ENTRY_NUM)

  // s0: hash pc -> cam all entries
  val s0_can_accept = Wire(Bool())
  val s0_valid = io.req_val
  val s0_vaddr = io.req_vaddr
  val s0_paddr = io.req_addr
  val s0_pc = io.req_pc
  val s0_pc_hash = pc_hash_tag(s0_pc)
  val s0_pc_match_vec = VecInit(array zip valids map { case (e, v) => e.tag_match(v, s0_valid, s0_pc_hash) }).asUInt
  val s0_hit = s0_pc_match_vec.orR
  val s0_index = Mux(s0_hit, OHToUInt(s0_pc_match_vec), replacement.way)

  when(s0_valid) {
    replacement.access(s0_index)
  }

  assert(PopCount(s0_pc_match_vec) <= 1.U)

  // s1: alloc or update
  val s1_valid = RegNext(s0_valid && s0_vaddr =/= 0.U)
  val s1_index = RegEnable(s0_index, s0_valid)
  val s1_pc_hash = RegEnable(s0_pc_hash, s0_valid)
  val s1_vaddr = RegEnable(s0_vaddr, s0_valid)
  val s1_paddr = RegEnable(s0_paddr, s0_valid)
  val s1_hit = RegEnable(s0_hit, s0_valid)
  val s1_alloc = s1_valid && !s1_hit
  val s1_update = s1_valid && s1_hit
  val s1_stride = array(s1_index).stride
  val s1_new_stride = WireInit(0.U(STRIDE_BITS.W))
  val s1_can_send_pf = WireInit(false.B)
  s0_can_accept := !(s1_valid && s1_pc_hash === s0_pc_hash)

  val always_update = ALWAYS_UPDATE_PRE_VADDR.B

  when(s1_alloc) {
    valids(s1_index) := true.B
    array(s1_index).alloc(
      vaddr = s1_vaddr,
      alloc_hash_pc = s1_pc_hash
    )
  }.elsewhen(s1_update) {
    val res = array(s1_index).update(s1_vaddr, always_update)
    s1_can_send_pf := res._1
    s1_new_stride := res._2
  }

  val stride_ratio = STRIDE_DEPTH_RATIO.U
  // s2: calculate L1 & L2 pf physical addr
  val s2_valid = RegNext(s1_valid && s1_can_send_pf)
  val s2_vaddr = RegEnable(s1_vaddr, s1_valid && s1_can_send_pf)
  val s2_paddr = RegEnable(s1_paddr, s1_valid && s1_can_send_pf)
  val s2_stride = RegEnable(s1_stride, s1_valid && s1_can_send_pf)
  val s2_depth = s2_stride << stride_ratio
  val s2_pf_vaddr = (s2_vaddr + s2_depth)(vaddrBits - 1, 0)
  val s2_pf_paddr = (s2_paddr + s2_depth)(paddrBits - 1, 0)
  def samePage(a: UInt, b: UInt): Bool = {
    a(vaddrBits - 1, PAGE_OFFSET) === b(vaddrBits - 1, PAGE_OFFSET)
  }
  val s2_pf_paddr_valid = s2_valid && samePage(s2_pf_vaddr, s2_vaddr)

  // TODO: virtual address prefetch request
  /*
  io.prefetch_translation_req.valid := s2_valid
  io.prefetch_translation_req.bits.translation_vaddr := s2_pf_vaddr
  io.prefetch_translation_resp.ready := s2_valid

  val s2_trans_paddr = RegInit(0.U(coreMaxAddrBits.W))
  val s2_pf_valid = RegInit(false.B)

  // 处理翻译结果
  when (io.prefetch_translation_resp.valid) {
    when (!io.prefetch_translation_resp.bits.translation_miss) {
      s2_trans_paddr := io.prefetch_translation_resp.bits.translation_paddr
    } .otherwise {
      // 翻译失败，取消预取
      s2_pf_valid := false.B
    }

  }

  // s3: send vaddr prefetch request
  val s3_valid = RegNext(s2_valid && io.prefetch_translation_resp.valid && !io.prefetch_translation_resp.bits.translation_miss)
  */
  io.prefetch.valid := s2_pf_paddr_valid && io.mshr_avail
  io.prefetch.bits.addr := s2_pf_paddr
  io.prefetch.bits.uop := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := M_PFR
  io.prefetch.bits.data := DontCare
  io.prefetch.bits.vaddr := s2_pf_vaddr
  io.prefetch.bits.is_hella := false.B

  io.prefetch_translation_req.valid := false.B
  io.prefetch_translation_req.bits.translation_vaddr := DontCare
  io.prefetch_translation_resp.ready := false.B



/*
 * TODO: support flush prefetcher
  for(i <- 0 until STRIDE_ENTRY_NUM) {
    when(GatedValidRegNext(io.flush)) {
      reset_array(i)
    }
  }
*/

}