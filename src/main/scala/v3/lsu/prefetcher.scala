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


// Prefetch type constants for tracking which prefetcher issued the request
object PrefetchType {
  val NULL_PREFETCH   = 0.U(2.W)  // Not a prefetch or unknown source
  val NL_PREFETCH     = 1.U(2.W)  // Next-line prefetcher
  val STRIDE_PREFETCH = 2.U(2.W)  // Stride prefetcher
  val STREAM_PREFETCH = 3.U(2.W)  // Stream prefetcher
}

abstract class DataPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends BoomModule()(p)
{
  val io = IO(new Bundle {
    val mshr_avail = Input(Bool())
    val req_val    = Input(Bool())
    val req_addr   = Input(UInt(coreMaxAddrBits.W))
    val req_vaddr  = Input(UInt(coreMaxAddrBits.W))
    val req_coh    = Input(new ClientMetadata)
    val req_pc     = Input(UInt(coreMaxAddrBits.W))
    val req_miss   = Input(Bool())
    val req_pfHit  = Input(UInt(2.W)) // 0: no pf hit, 1: nl pf hit, 2: stride pf hit, 3: stream pf hit

    val prefetch   = Decoupled(new BoomDCacheReq)
    // Prefetch type for tracking which prefetcher issued this request
    val prefetch_type = Output(UInt(2.W))

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
  io.prefetch_type  := PrefetchType.NULL_PREFETCH

  // 默认不需要进行地址翻译
  io.prefetch_translation_req.valid := false.B
  io.prefetch_translation_req.bits.translation_vaddr := DontCare
  io.prefetch_translation_resp.ready := false.B
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
  when (io.req_val && cacheable && io.req_miss) {
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
  io.prefetch_type             := PrefetchType.NL_PREFETCH
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
  io.prefetch_type             := PrefetchType.NL_PREFETCH
}


trait HasL1PrefetcherHelper extends HasL1HellaCacheParameters {
  // region related
  // 每个region为1024B
  val REGION_SIZE = 1024
  val PAGE_OFFSET = 12
  // region内的块数量 -> 16 个 cache line
  val BIT_VEC_WIDTH = REGION_SIZE / cacheBlockBytes
  // 4 bit
  val REGION_BITS = log2Up(BIT_VEC_WIDTH)
  // 10 bit offset
  val REGION_TAG_OFFSET = lgCacheBlockBytes + REGION_BITS
  // 16 bit tag
  val REGION_TAG_BITS = vaddrBits - lgCacheBlockBytes - REGION_BITS

  // hash related
  val VADDR_HASH_WIDTH = 5
  val BLK_ADDR_RAW_WIDTH = 10
  val HASH_TAG_WIDTH = VADDR_HASH_WIDTH + BLK_ADDR_RAW_WIDTH

  // capacity related
  val FILTER_REGION_SIZE = 8

  // prefetch sink related
  // val SINK_BITS = 2
  // def SINK_L1 = "b00".U
  // def SINK_L2 = "b01".U
  // def SINK_L3 = "b10".U

  // vaddr: |       region tag        |  region bits  | block offset |
  def get_region_tag(vaddr: UInt) = {
    require(vaddr.getWidth >= vaddrBits)
    vaddr(vaddr.getWidth - 1, REGION_TAG_OFFSET)
  }

  def get_region_bits(vaddr: UInt) = {
    require(vaddr.getWidth >= vaddrBits)
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

  // 这里有点小问题：判断同一页的话，应该还是要用虚拟地址比较合理，但是在stream里没太大差别
  def samePage(a: UInt, b: UInt): Bool = {
    a(paddrBits - 1, PAGE_OFFSET) === b(paddrBits - 1, PAGE_OFFSET)
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

// =============================================================================
// Stride Prefetcher
// =============================================================================

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
  val s0_valid = io.req_val && (io.req_miss || io.req_pfHit === PrefetchType.STRIDE_PREFETCH)
  val s0_vaddr = io.req_vaddr
  val s0_paddr = io.req_addr
  val s0_pc = io.req_pc
  val s0_pc_hash = pc_hash_tag(s0_pc)
  // declare s1 registers early so they exist during elaboration
  val s1_valid = RegInit(false.B)
  val s1_index = RegInit(0.U(log2Up(STRIDE_ENTRY_NUM).W))
  val s1_pc_hash = RegInit(0.U(s0_pc_hash.getWidth.W))
  val s1_vaddr = RegInit(0.U(s0_vaddr.getWidth.W))
  val s1_paddr = RegInit(0.U(s0_paddr.getWidth.W))
  val s1_hit = RegInit(false.B)

  val s0_pc_match_vec = VecInit(array zip valids map { case (e, v) => e.tag_match(v, s0_valid, s0_pc_hash) }).asUInt

  val s0_s1_match = s0_valid && s1_valid && s0_pc_hash === s1_pc_hash
  val s0_hit = s0_s1_match | s0_pc_match_vec.orR
  val s0_index = Mux(s0_s1_match, s1_index, Mux(s0_hit, OHToUInt(s0_pc_match_vec), replacement.way))

  when(s0_valid) {
    replacement.access(s0_index)
    // printf("StridePrefetcher s0_valid event:\n")
    // for(i <- 0 until STRIDE_ENTRY_NUM) {
    //   printf("StridePrefetcher array(%d) status: pre_vaddr = %x, stride = %x, confidence = %d, hash_pc = %x, valid = %d\n", i.U, array(i).pre_vaddr, array(i).stride, array(i).confidence, array(i).hash_pc, valids(i))
    // }
    // printf("StridePrefetcher status: s0_pc = %x, s0_pc_hash = %x, s0_hit = %d, s0_index = %d, s0_pc_match_vec = %x\n\n", s0_pc, s0_pc_hash, s0_hit, s0_index, s0_pc_match_vec)
  }

  assert(PopCount(s0_pc_match_vec) <= 1.U)

  // s1: alloc or update
  // Replace RegNext/RegEnable patterns with explicit RegInit + conditional updates
  // to avoid forward-reference issues that can produce null chisel nodes.

  when (s0_valid) {
    s1_valid := s0_vaddr =/= 0.U
    s1_index := s0_index
    s1_pc_hash := s0_pc_hash
    s1_vaddr := s0_vaddr
    s1_paddr := s0_paddr
    s1_hit := s0_hit
  } .otherwise {
    s1_valid := false.B
  }

  val s1_alloc = s1_valid && !s1_hit
  val s1_update = s1_valid && s1_hit
  val s1_stride = array(s1_index).stride
  val s1_new_stride = WireInit(0.U(STRIDE_BITS.W))
  val s1_can_send_pf = WireInit(false.B)
  s0_can_accept := !(s1_valid && s1_pc_hash === s0_pc_hash)

  // when(s1_valid) {
  //   printf("StridePrefetcher s1_valid event:\n")
  //   for(i <- 0 until STRIDE_ENTRY_NUM) {
  //     printf("StridePrefetcher array(%d) status: pre_vaddr = %x, stride = %x, confidence = %d, hash_pc = %x, valid = %d\n", i.U, array(i).pre_vaddr, array(i).stride, array(i).confidence, array(i).hash_pc, valids(i))
  //   }
  //   printf("StridePrefetcher status: s1_pc_hash = %x, s1_hit = %d, s1_index = %d, s1_paddr = %x, s1_vaddr = %x\n\n", s1_pc_hash, s1_hit, s1_index, s1_paddr, s1_vaddr)
  // }

  val always_update = ALWAYS_UPDATE_PRE_VADDR.B

  when(s1_alloc) {
    valids(s1_index) := true.B
    array(s1_index).alloc(
      vaddr = s1_vaddr,
      alloc_hash_pc = s1_pc_hash
    )
    // printf("StridePrefetcher s1_alloc event: array(s1_index = %d) <-  s1_pc_hash = %x, vaddr = %x\n", s1_index, s1_pc_hash, s1_vaddr)
  }.elsewhen(s1_update) {
    val res = array(s1_index).update(s1_vaddr, always_update)
    s1_can_send_pf := res._1
    s1_new_stride := res._2
    // printf("StridePrefetcher s1_update event: array(s1_index = %d) updated with s1_pc_hash = %x, vaddr = %x, can_send_pf = %d, new_stride = %x\n", s1_index, s1_pc_hash, s1_vaddr, res._1, res._2)
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
  val cacheable = edge.manager.supportsAcquireBSafe(s2_pf_paddr, lgCacheBlockBytes.U)
  io.prefetch.valid := s2_pf_paddr_valid && io.mshr_avail && cacheable
  io.prefetch.bits.addr := s2_pf_paddr
  io.prefetch.bits.uop := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := M_PFR
  io.prefetch.bits.data := DontCare
  io.prefetch.bits.vaddr := s2_pf_vaddr
  io.prefetch.bits.is_hella := false.B
  io.prefetch_type := PrefetchType.STRIDE_PREFETCH

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

// =============================================================================
// Stream Prefetcher
// =============================================================================

trait HasStreamPrefetchHelper extends HasL1PrefetcherHelper {
  // capacity related
  // 流数量
  val STREAM_FILTER_SIZE = 4
  // 流监控区域数量
  val BIT_VEC_ARRAY_SIZE = 16
  // 活跃阈值
  val ACTIVE_THRESHOLD = BIT_VEC_WIDTH - 4
  val INIT_DEC_MODE = false

  // bit_vector [StreamBitVectorBundle]:
  // `X`: valid; `.`: invalid; `H`: hit
  // [X X X X X X X X X . . H . X X X]                                                         [. . X X X X . . . . . . . . . .]
  //                    hit in 12th slot & active           --------------------->             prefetch bit_vector [StreamPrefetchReqBundle]
  //                        |  <---------------------------- depth ---------------------------->
  //                                                                                           | <-- width -- >
  // 流预取深度和宽度
  val DEPTH_BYTES = 1024
  val DEPTH_CACHE_BLOCKS = DEPTH_BYTES / cacheBlockBytes
  val WIDTH_BYTES = 256
  val WIDTH_CACHE_BLOCKS = WIDTH_BYTES / cacheBlockBytes

  // val DEPTH_LOOKAHEAD = 6
  // val DEPTH_BITS = log2Up(DEPTH_CACHE_BLOCKS) + DEPTH_LOOKAHEAD

  val ENABLE_DECR_MODE = false
  val ENABLE_STRICT_ACTIVE_DETECTION = true

  // constraints
  require((DEPTH_BYTES >= REGION_SIZE) && ((DEPTH_BYTES % REGION_SIZE) == 0) && ((DEPTH_BYTES / REGION_SIZE) > 0))
  require(((VADDR_HASH_WIDTH * 3) + BLK_ADDR_RAW_WIDTH) <= REGION_TAG_BITS)
  require(WIDTH_BYTES >= cacheBlockBytes)
}

// 区域位向量
class StreamBitVectorBundle(implicit p: Parameters) extends BoomBundle with HasStreamPrefetchHelper {
  // 区域标签
  val tag = UInt(REGION_TAG_BITS.W)
  // bit向量
  val bit_vec = UInt(BIT_VEC_WIDTH.W)
  // 标记活跃区域
  val active = Bool()
  // cnt can be optimized
  // 计数器
  val cnt = UInt((log2Up(BIT_VEC_WIDTH) + 1).W)
  // 流方向
  val decr_mode = Bool()

  // debug usage
  // val trigger_full_va = UInt(vaddrBits.W)

  def reset(index: Int) = {
    tag := index.U
    bit_vec := 0.U
    active := false.B
    cnt := 0.U
    decr_mode := INIT_DEC_MODE.B
    // trigger_full_va := 0xdeadbeefL.U
  }

  def tag_match(valid1: Bool, valid2: Bool, new_tag: UInt): Bool = {
    // 哈希结果匹配
    valid1 && valid2 && region_hash_tag(tag) === region_hash_tag(new_tag)
  }

  // 针对新区域分配条目
  def alloc(alloc_tag: UInt, alloc_bit_vec: UInt, alloc_active: Bool, alloc_decr_mode: Bool) = {
    tag := alloc_tag
    bit_vec := alloc_bit_vec
    active := alloc_active
    cnt := 1.U
    // trigger_full_va := alloc_full_vaddr
    if(ENABLE_DECR_MODE) {
      decr_mode := alloc_decr_mode
    }else {
      decr_mode := INIT_DEC_MODE.B
    }


    assert(PopCount(alloc_bit_vec) === 1.U, "alloc vector should be one hot")
  }

  // 更新位向量
  def update(update_bit_vec: UInt, update_active: Bool) = {
    // if the slot is 0 before, increment cnt
    // only update_bit_vec & bit_vec == 0 -> cnt++
    // which means update_bit_vec is a not hit before cache line
    val cnt_en = !((bit_vec & update_bit_vec).orR)
    val cnt_next = Mux(cnt_en, cnt + 1.U, cnt)

    // 更新位向量和计数器
    bit_vec := bit_vec | update_bit_vec
    cnt := cnt_next
    // 活跃度检测
    when(cnt_next >= ACTIVE_THRESHOLD.U) {
      active := true.B
    }
    when(update_active) {
      active := true.B
    }

    assert(PopCount(update_bit_vec) === 1.U, "update vector should be one hot")
    assert(cnt <= BIT_VEC_WIDTH.U, "cnt should always less than bit vector size")
  }
}

class StreamPrefetchReqBundle(implicit p: Parameters) extends BoomBundle with HasStreamPrefetchHelper {
  val addr = UInt(coreMaxAddrBits.W)
  val cnt  = UInt((log2Up(WIDTH_CACHE_BLOCKS) + 1).W)
  val decr_mode = Bool()

  def getStreamPrefetchReqBundle(addr: UInt, width: Int, decr_mode: Bool): StreamPrefetchReqBundle = {
    val bundle = Wire(new StreamPrefetchReqBundle)
    bundle.addr := addr
    bundle.cnt := width.U
    bundle.decr_mode := decr_mode
    bundle
  }
}


/* 
  * Stream Prefetcher
  * 类似于IPCP中的GS预取器，即Global Stream Prefetcher
  * 核心机制是检查密集访问的区域，在对密集访问区域进行请求时，预取其前进或后退方向的其他stream
  * 工作流程:
  *   s0: 取出数据，区域标签匹配，检查命中
  *   s1: 分配或更新，训练区域活跃度信息，这里同时也会判断当前请求是否命中活跃区域
  *   s2: 发送stream请求
*/
class StreamPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher with HasStreamPrefetchHelper {
  // 16-entry bit vector array
  val array = Reg(Vec(BIT_VEC_ARRAY_SIZE, new StreamBitVectorBundle))
  val valids = RegInit(VecInit(Seq.fill(BIT_VEC_ARRAY_SIZE)(false.B)))

  def reset_array(i: Int): Unit = {
    valids(i) := false.B
    //only need to rest control signals for firendly area
    // array(i).reset(i)
  }

  val replacement = ReplacementPolicy.fromString("plru", BIT_VEC_ARRAY_SIZE)

  //==============================================
  //========== Stage 0: Region Tag Match =========
  //==============================================
  // s0: generate region tag, parallel match
  // CAM并行比较
  // val s0_can_accept = Wire(Bool())
  // 提前声明涉及到前递的信号，避免空节点编译错误
  val s1_valid = RegInit(false.B)
  val s1_index = RegInit(0.U(log2Up(BIT_VEC_ARRAY_SIZE).W))
  val s1_region_tag = RegInit(0.U(REGION_TAG_BITS.W))
  val s1_hit = RegInit(false.B)
  // val s1_miss  = RegInit(false.B)
  // val s1_pfHit = RegInit(false.B)

  val s0_valid = io.req_val && (io.req_vaddr =/= 0.U)
  val s0_pc    = io.req_pc
  val s0_paddr  = io.req_addr
  val s0_vaddr = io.req_vaddr
  val s0_miss  = io.req_miss
  val s0_pfHit = io.req_pfHit === PrefetchType.STREAM_PREFETCH
  // TODO:训练请求类型: 原则上, Stream Prefetcher对所有需求请求都会训练，但只会对miss和pfHitStream触发预取
  // val s0_miss  = io.train_req.bits.miss
  // val s0_pfHit = io.train_req.bits.pfHitStream
  // 计算Region Tag & Region Bits
  val s0_region_bits = get_region_bits(s0_vaddr)
  val s0_region_tag = get_region_tag(s0_vaddr)
  val s0_region_tag_plus_one = get_region_tag(s0_vaddr) + 1.U
  val s0_region_tag_minus_one = get_region_tag(s0_vaddr) - 1.U
  // 区域匹配（读array和valids）
  val s0_region_tag_match_vec = array zip valids map { case (e, v) => e.tag_match(v, s0_valid, s0_region_tag) }
  val s0_region_tag_plus_one_match_vec = array zip valids map { case (e, v) => e.tag_match(v, s0_valid, s0_region_tag_plus_one) }
  val s0_region_tag_minus_one_match_vec = array zip valids map { case (e, v) => e.tag_match(v, s0_valid, s0_region_tag_minus_one) }


  // 冒险检测信号
  // 同一个region，s1会alloc或update该region的统计信息
  val s0_s1_match_alloc = s1_valid && (region_hash_tag(s1_region_tag) === region_hash_tag(s0_region_tag)) && !s1_hit
  // 区域命中结果
  val s0_hit = Cat(s0_region_tag_match_vec).orR || s0_s1_match_alloc
  val s0_plus_one_hit = Cat(s0_region_tag_plus_one_match_vec).orR
  val s0_minus_one_hit = Cat(s0_region_tag_minus_one_match_vec).orR
  // 命中索引
  val s0_hit_vec = VecInit(s0_region_tag_match_vec).asUInt
  val s0_index = Mux(s0_s1_match_alloc, s1_index, Mux(s0_hit, OHToUInt(s0_hit_vec), replacement.way))
  val s0_plus_one_index = OHToUInt(VecInit(s0_region_tag_plus_one_match_vec).asUInt)
  val s0_minus_one_index = OHToUInt(VecInit(s0_region_tag_minus_one_match_vec).asUInt)
  // io.train_req.ready := s0_can_accept

  // 替换策略获取结果
  when(s0_valid) {
    replacement.access(s0_index)
  }

  assert(!s0_valid || PopCount(VecInit(s0_region_tag_match_vec)) <= 1.U, "req region should match no more than 1 entry")
  assert(!s0_valid || PopCount(VecInit(s0_region_tag_plus_one_match_vec)) <= 1.U, "req region plus 1 should match no more than 1 entry")
  assert(!s0_valid || PopCount(VecInit(s0_region_tag_minus_one_match_vec)) <= 1.U, "req region minus 1 should match no more than 1 entry")
  assert(!s0_valid || !(s0_hit && s0_plus_one_hit && (s0_index === s0_plus_one_index)), "region and region plus 1 index match failed")
  assert(!s0_valid || !(s0_hit && s0_minus_one_hit && (s0_index === s0_minus_one_index)), "region and region minus 1 index match failed")
  assert(!s0_valid || !(s0_plus_one_hit && s0_minus_one_hit && (s0_minus_one_index === s0_plus_one_index)), "region plus 1 and region minus 1 index match failed")
  assert(!(s0_valid && RegNext(s0_valid) && !s0_hit && !RegEnable(s0_hit, s0_valid) && replacement.way === RegEnable(replacement.way, s0_valid)), "replacement error")

  //==============================================
  //========== Stage 1: Alloc or Update ==========
  //==============================================
  
  when (s0_valid) {
    s1_valid := true.B
    s1_index := s0_index
    s1_hit := s0_hit
    s1_region_tag := s0_region_tag
  } .otherwise {
    s1_valid := false.B
  }
  val s1_alloc = s1_valid && !s1_hit
  val s1_update = s1_valid && s1_hit
  val s1_pc    = RegEnable(s0_pc, s0_valid)
  val s1_paddr = RegEnable(s0_paddr, s0_valid)
  val s1_vaddr = RegEnable(s0_vaddr, s0_valid)
  val s1_miss  = RegEnable(s0_miss, s0_valid)
  val s1_pfHit = RegEnable(s0_pfHit, s0_valid)
  val s1_plus_one_index = RegEnable(s0_plus_one_index, s0_valid)
  val s1_minus_one_index = RegEnable(s0_minus_one_index, s0_valid)
  val s1_plus_one_hit = if(ENABLE_STRICT_ACTIVE_DETECTION)
                            RegEnable(s0_plus_one_hit, s0_valid) && array(s1_plus_one_index).active && (array(s1_plus_one_index).cnt >= ACTIVE_THRESHOLD.U)
                        else
                            RegEnable(s0_plus_one_hit, s0_valid) && array(s1_plus_one_index).active
  val s1_minus_one_hit = if(ENABLE_STRICT_ACTIVE_DETECTION)
                            RegEnable(s0_minus_one_hit, s0_valid) && array(s1_minus_one_index).active && (array(s1_minus_one_index).cnt >= ACTIVE_THRESHOLD.U)
                        else
                            RegEnable(s0_minus_one_hit, s0_valid) && array(s1_minus_one_index).active
  val s1_region_bits = RegEnable(s0_region_bits, s0_valid)
  // TODO: 支持动态深度
  val s1_pf_incr_vaddr = Cat(region_to_block_addr(s1_region_tag, s1_region_bits) + DEPTH_CACHE_BLOCKS.U, 0.U(lgCacheBlockBytes.W))
  val s1_pf_decr_vaddr = Cat(region_to_block_addr(s1_region_tag, s1_region_bits) - DEPTH_CACHE_BLOCKS.U, 0.U(lgCacheBlockBytes.W))
  // TODO: 添加预取训练请求数据来源追踪
  // val strict_trigger_const = Constantin.createRecord(s"StreamStrictTrigger_${p(XSCoreParamsKey).HartId}", initValue = 1)
  // If use strict triggering mode, the stream prefetcher will only trigger prefetching
  // under **cache miss or prefetch hit stream**, but will still perform training on the entire memory access trace.
  // val s1_can_trigger = Mux(strict_trigger_const.orR, s1_miss || s1_pfHit, true.B)
  val s1_can_trigger = s1_miss || s1_pfHit
  // 一个区域中的一次访问，仅有首次访问会触发预取请求
  val s1_can_send_pf = Mux(s1_update, !((array(s1_index).bit_vec & UIntToOH(s1_region_bits)).orR), true.B) && s1_can_trigger
  // 如果可能出现array的数据冒险，反压s0阶段，目前暂时没做出来
  // TODO: 提供输入请求握手，允许通过s0_can_accept进行反压

  when(s1_alloc) {
    // alloc a new entry
    valids(s1_index) := true.B
    array(s1_index).alloc(
      alloc_tag = s1_region_tag,
      alloc_bit_vec = UIntToOH(s1_region_bits),
      alloc_active = s1_plus_one_hit || s1_minus_one_hit,
      alloc_decr_mode = RegEnable(s0_plus_one_hit, s0_valid)
      // alloc_full_vaddr = RegEnable(s0_vaddr, s0_valid)
    )
  }.elsewhen(s1_update) {
    // update a existing entry
    assert(array(s1_index).cnt =/= 0.U || valids(s1_index), "entry should have been allocated before")
    array(s1_index).update(
      update_bit_vec = UIntToOH(s1_region_bits),
      update_active = s1_plus_one_hit || s1_minus_one_hit
    )
  }

  // s2: trigger prefetch if hit active bit vector, compute meta of prefetch req
  val s2_valid = RegNext(s1_valid)
  val s2_index = RegEnable(s1_index, s1_valid)
  val s2_pc    = RegEnable(s1_pc, s1_valid)
  val s2_paddr = RegEnable(s1_paddr, s1_valid)
  val s2_vaddr = RegEnable(s1_vaddr, s1_valid)
  val s2_region_bits = RegEnable(s1_region_bits, s1_valid)
  val s2_region_tag = RegEnable(s1_region_tag, s1_valid)
  val s2_pf_incr_vaddr = RegEnable(s1_pf_incr_vaddr, s1_valid)
  val s2_pf_decr_vaddr = RegEnable(s1_pf_decr_vaddr, s1_valid)
  val s2_can_send_pf = RegEnable(s1_can_send_pf, s1_valid)
  val s2_can_trigger = RegEnable(s1_can_trigger, s1_valid)
  // 这里可能存在冒险：s1阶段更新了active位，s2阶段读的时候是最新或老的active位判断？
  // 但应该不会对结果造成太大影响
  val s2_active = array(s2_index).active
  val s2_decr_mode = array(s2_index).decr_mode
  val s2_pf_vaddr = Mux(s2_decr_mode, s2_pf_decr_vaddr, s2_pf_incr_vaddr)
  val s2_pf_paddr = Cat(s2_paddr(s2_paddr.getWidth - 1, pgIdxBits), s2_pf_vaddr(pgIdxBits - 1, 0))


  val cacheable = edge.manager.supportsAcquireBSafe(s2_pf_paddr, lgCacheBlockBytes.U)
  val s2_will_send_pf = s2_valid && s2_active && s2_can_send_pf && samePage(s2_pf_vaddr, s2_vaddr) && cacheable

  //TODO: enable signal
  val s2_pf_req_valid = s2_will_send_pf && s2_can_trigger

  val s2_pf_req_bits = (new StreamPrefetchReqBundle).getStreamPrefetchReqBundle(
    addr = s2_pf_paddr,
    width = WIDTH_CACHE_BLOCKS,
    decr_mode = s2_decr_mode
  )


  // s3: send the l1 prefetch req out
  val stream_engine = Module(new StreamPrefetchEngine)
  stream_engine.io.l1_prefetch_req.valid := s2_pf_req_valid
  stream_engine.io.l1_prefetch_req.bits := s2_pf_req_bits
  io.prefetch <> stream_engine.io.prefetch
  io.prefetch_type := PrefetchType.STREAM_PREFETCH
  io.prefetch_translation_req.valid := false.B
  io.prefetch_translation_req.bits.translation_vaddr := DontCare
  io.prefetch_translation_resp.ready := false.B

/*
TODO: support flush prefetcher
  // reset meta to avoid muti-hit problem
  for(i <- 0 until BIT_VEC_ARRAY_SIZE) {
    when(GatedValidRegNext(io.flush)) {
      reset_array(i)
    }
  }

*/
}

class StreamPrefetchEngine(implicit p: Parameters, implicit val edge: TLEdgeOut) extends BoomModule with HasStreamPrefetchHelper {
  val io = IO(new Bundle {
    val l1_prefetch_req = Flipped(Decoupled(new StreamPrefetchReqBundle))
    val prefetch        = Decoupled(new BoomDCacheReq)
  })

  // ------------------------------------------------------------------
  // 1. FIFO：只缓存“原始 stream 请求”
  // ------------------------------------------------------------------
  val fifo = Module(new Queue(new StreamPrefetchReqBundle, entries = 4, flow = true))
  fifo.io.enq <> io.l1_prefetch_req

  // ------------------------------------------------------------------
  // 2. 当前 active stream 的状态寄存器
  // ------------------------------------------------------------------
  val valid = RegInit(false.B)
  val cur_addr   = Reg(UInt(paddrBits.W))
  val cnt    = Reg(UInt(log2Up(WIDTH_CACHE_BLOCKS + 1).W))
  val decr_mode  = Reg(Bool())

  // 下一个 cache line 地址
  val next_addr = Mux(
    decr_mode,
    cur_addr - cacheBlockBytes.U,
    cur_addr + cacheBlockBytes.U
  )

  // 当前这一次 fire 是否是最后一次
  val cacheable = edge.manager.supportsAcquireBSafe(next_addr, lgCacheBlockBytes.U)
  val last_fire = (cnt === 1.U) || !samePage(cur_addr, next_addr) || !cacheable
  val can_chain = last_fire && io.prefetch.fire

  // ------------------------------------------------------------------
  // 3. 从 FIFO 取新 stream
  // ------------------------------------------------------------------
  // FIFO 出队：当我们“接收”了一个新 stream
  fifo.io.deq.ready := !valid || can_chain
  when (fifo.io.deq.fire) {
    // 取出一个新的 stream
    cur_addr  := fifo.io.deq.bits.addr
    cnt       := fifo.io.deq.bits.cnt
    decr_mode := fifo.io.deq.bits.decr_mode
    valid     := true.B
  }

  // ------------------------------------------------------------------
  // 4. Prefetch 输出接口
  // ------------------------------------------------------------------
  io.prefetch.valid := valid

  io.prefetch.bits.addr := cur_addr
  io.prefetch.bits.uop  := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := M_PFR
  io.prefetch.bits.data := DontCare
  io.prefetch.bits.is_hella := false.B
  io.prefetch.bits.vaddr := DontCare // not used in L1 prefetcher

  // ------------------------------------------------------------------
  // 5. fire 时推进 stream
  // ------------------------------------------------------------------
  when (io.prefetch.fire) {
    when (last_fire) {
      // 当前这拍发完就结束 stream
      when (fifo.io.deq.valid) {
        // 还能接下一个 stream
        cur_addr  := fifo.io.deq.bits.addr
        cnt   := fifo.io.deq.bits.cnt
        decr_mode := fifo.io.deq.bits.decr_mode
        valid    := true.B
      }.otherwise {
        // 没有下一个 stream 了
        valid := false.B
      }
    }.otherwise {
      // 还能继续发下一个 cache line
      cur_addr := next_addr
      cnt  := cnt - 1.U
    }
  }
}