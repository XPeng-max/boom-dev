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


/**
  * PrefetchFilter IO Bundle
  * 预取过滤器的标准接口
  */
class PrefetchFilterIO(implicit p: Parameters) extends BoomBundle with HasSandboxFilterParameters {
  // 预取请求输入
  val prefetch_in  = Flipped(Decoupled(new BoomDCacheReq))
  val prefetch_type_in = Input(UInt(3.W))
  // val refetch_pc_hash_in = Input(UInt(HASH_TAG_WIDTH.W))
  // 预取请求输出
  val prefetch_out = Decoupled(new BoomDCacheReq)
  val prefetch_type_out = Output(UInt(3.W))
  // val prefetch_pc_hash_out = Output(UInt(HASH_TAG_WIDTH.W))
}


/**
  * Abstract PrefetchFilter base class
  * 预取过滤器抽象基类
  * 
  * 子类可以实现不同的过滤策略：
  * - NullPrefetchFilter: 直连，不做过滤
  * - SandboxPrefetchFilter: 基于地址表的重复过滤，支持 demand 查询
  * - BitVecPrefetchFilter: 基于区域位向量的过滤
  * - BloomPrefetchFilter(TODO): 基于布隆过滤器的复杂过滤器
  */
abstract class PrefetchFilter(implicit p: Parameters) extends BoomModule {
  val io = IO(new PrefetchFilterIO)
}

/**
  * NullPrefetchFilter - 直连过滤器
  * 不做任何过滤，直接将输入连接到输出
  * demand_in 和 sample_update 信号被忽略
  */
class NullPrefetchFilter(implicit p: Parameters) extends PrefetchFilter {
  io.prefetch_out <> io.prefetch_in
  io.prefetch_type_out := io.prefetch_type_in
  // io.prefetch_pc_hash_out := io.prefetch_pc_hash_in
  
  // Debug: 直连过滤器，打印所有通过的预取请求
  when (io.prefetch_out.fire) {
    printf(p"[NullPrefetchFilter] PASS: addr=0x${Hexadecimal(io.prefetch_out.bits.addr)}, prefetch_type=${io.prefetch_type_out}\n")
  }
}

/**
  * SandboxPrefetchFilter - 基于地址表的预取过滤器 (SyncReadMem/BRAM 实现)
  * 
  * 设计选择说明：
  * 1. 存储结构：SyncReadMem (FPGA上映射为BRAM)
  *    - 优点：节省LUT/FF，利用闲置BRAM资源
  *    - 缺点：1周期读延迟，需要RAW bypass逻辑
  * 
  * 2. 映射方式：直接映射
  *    - 优点：硬件开销最小，单次访问只需1次比较
  *    - 缺点：冲突率较高，但对于prefetch filter可接受
  *    - 替代方案：2-way组相联可降低冲突，但增加比较器和替换逻辑
  * 
  * 3. 容量选择：可配置，默认512 entries
  *    - 确保在Xilinx FPGA上映射到BRAM（≥18Kb threshold）
  * 
  * 工作流程：
  *   S0: 接收请求，发起BRAM读
  *   S1: 比较tag判断命中，bypass处理RAW冒险，输出或过滤
  */

trait HasSandboxFilterParameters extends HasL1PrefetcherHelper {
  // 可配置参数
  val SANDBOX_TABLE_SIZE = 512
  val SANDBOX_TAG_WIDTH = coreMaxAddrBits - lgCacheBlockBytes - log2Ceil(SANDBOX_TABLE_SIZE)
  val SANDBOX_IDX_WIDTH = log2Ceil(SANDBOX_TABLE_SIZE)
}

class SandboxEntry(implicit p: Parameters) extends BoomBundle with HasSandboxFilterParameters {
  val valid = Bool()
  val tag = UInt(SANDBOX_TAG_WIDTH.W)
  val pc_hash = UInt(HASH_TAG_WIDTH.W)
  val prefetch_type = UInt(3.W)  // 记录是哪种预取器发出的请求
}
/**
  * BitVecPrefetchFilter - 基于区域位向量的预取过滤器
  * 
  * 设计说明：
  * 1. 使用 BITVEC_FILTER_SIZE 个 entry，每个 entry 覆盖 FILTER_REGION_SIZE 个 cache block
  * 2. 每个 entry 包含：
  *    - tag: 区域标签，用于匹配地址的高位
  *    - bitvec: 位向量，记录区域内哪些 cache block 已被预取
  * 3. 过滤逻辑：
  *    - 命中 entry 且对应 bit 已置位 -> 过滤（已预取过）
  *    - 命中 entry 但对应 bit 未置位 -> 放行并置位
  *    - 未命中 -> PLRU 分配新 entry，置位并放行
  * 
  * 优点：
  * - 比 SandboxPrefetchFilter 更高效利用存储空间
  * - 一个 entry 可以跟踪一个区域内多个 cache block 的预取状态
  * - 适合空间局部性较强的访问模式
  */

trait HasBitVecFilterParameters extends HasL1PrefetcherHelper {
  // 每个区域包含的 cache block 数量
  val FILTER_REGION_SIZE = 16
  // entry 数量
  val BITVEC_FILTER_SIZE = 16
  // tag 宽度：地址去掉 block offset 和 region bits
  val BITVEC_TAG_WIDTH = coreMaxAddrBits - lgCacheBlockBytes - log2Ceil(FILTER_REGION_SIZE)
  // region 内的 bit 索引宽度
  val BITVEC_REGION_BITS = log2Ceil(FILTER_REGION_SIZE)
}

class BitVecEntry(implicit p: Parameters) extends BoomBundle with HasBitVecFilterParameters {
  val tag = UInt(BITVEC_TAG_WIDTH.W)
  val bitvec = UInt(FILTER_REGION_SIZE.W)
}

class BitVecPrefetchFilter(implicit p: Parameters) extends PrefetchFilter with HasBitVecFilterParameters {
  
  // ========== 存储结构 ==========
  val entries = Reg(Vec(BITVEC_FILTER_SIZE, new BitVecEntry))
  val valids = RegInit(VecInit(Seq.fill(BITVEC_FILTER_SIZE)(false.B)))
  
  // PLRU 替换策略
  val replacement = ReplacementPolicy.fromString("plru", BITVEC_FILTER_SIZE)
  
  // ========== 地址解析 ==========
  def getTag(addr: UInt): UInt = addr(coreMaxAddrBits - 1, lgCacheBlockBytes + BITVEC_REGION_BITS)
  def getRegionBit(addr: UInt): UInt = addr(lgCacheBlockBytes + BITVEC_REGION_BITS - 1, lgCacheBlockBytes)
  
  // ========== S0: 接收请求，CAM 匹配 ==========
  val s0_valid = io.prefetch_in.valid
  val s0_addr = io.prefetch_in.bits.addr
  val s0_tag = getTag(s0_addr)
  val s0_region_bit = getRegionBit(s0_addr)
  val s0_bit_mask = UIntToOH(s0_region_bit)
  
  // 提前声明 S1 信号用于前递检测
  val s1_valid = RegInit(false.B)
  val s1_tag = Reg(UInt(BITVEC_TAG_WIDTH.W))
  val s1_index = Reg(UInt(log2Ceil(BITVEC_FILTER_SIZE).W))
  val s1_hit = Reg(Bool())
  
  // CAM 并行匹配
  val s0_match_vec = VecInit((entries zip valids).map { case (e, v) => 
    v && (e.tag === s0_tag)
  }).asUInt
  
  // S0-S1 前递：如果 S1 正在分配同一个 tag，视为命中
  val s0_s1_match = s1_valid && !s1_hit && (s1_tag === s0_tag)
  
  val s0_hit = s0_match_vec.orR || s0_s1_match
  val s0_hit_index = Mux(s0_s1_match, s1_index, OHToUInt(s0_match_vec))
  val s0_alloc_index = replacement.way
  val s0_index = Mux(s0_hit, s0_hit_index, s0_alloc_index)
  
  // S0 可接收条件
  val s0_can_accept = Wire(Bool())
  val s0_fire = s0_valid && s0_can_accept
  
  // ========== S1: 判断并更新 ==========
  val s1_bits = Reg(new BoomDCacheReq)
  val s1_type = Reg(UInt(3.W))
  // val s1_pc_hash = Reg(UInt(HASH_TAG_WIDTH.W))
  val s1_region_bit = Reg(UInt(BITVEC_REGION_BITS.W))
  val s1_bit_mask = Reg(UInt(FILTER_REGION_SIZE.W))
  
  // S0 -> S1 流水线推进
  when (s0_fire) {
    s1_valid := true.B
    s1_tag := s0_tag
    s1_index := s0_index
    s1_hit := s0_hit
    s1_bits := io.prefetch_in.bits
    s1_type := io.prefetch_type_in
    // s1_pc_hash := io.prefetch_pc_hash_in
    s1_region_bit := s0_region_bit
    s1_bit_mask := s0_bit_mask
    
    // 更新 PLRU
    replacement.access(s0_index)
  }
  
  // 读取当前 entry 的 bitvec（用于命中判断）
  val s1_entry_bitvec = entries(s1_index).bitvec
  
  // 判断对应 bit 是否已置位（需要过滤）
  val s1_bit_already_set = s1_hit && (s1_entry_bitvec & s1_bit_mask).orR
  
  // 需要过滤的条件：命中 entry 且对应 bit 已置位
  val s1_should_filter = s1_valid && s1_bit_already_set
  
  // ========== 输出逻辑 ==========
  io.prefetch_out.valid := s1_valid && !s1_should_filter
  io.prefetch_out.bits := s1_bits
  io.prefetch_type_out := s1_type
  // io.prefetch_pc_hash_out := s1_pc_hash
  
  // ========== Ready 信号 ==========
  val s1_will_complete = s1_should_filter || io.prefetch_out.fire
  s0_can_accept := !s1_valid || s1_will_complete
  io.prefetch_in.ready := s0_can_accept
  
  // ========== 写入/更新逻辑 ==========
  when (io.prefetch_out.fire) {
    when (s1_hit) {
      // 命中：更新 bitvec，置位对应 bit
      entries(s1_index).bitvec := s1_entry_bitvec | s1_bit_mask
      // Debug: 命中entry，放行并更新bitvec
      printf(p"[BitVecPrefetchFilter] PASS (hit): addr=0x${Hexadecimal(s1_bits.addr)}, prefetch_type=${s1_type}, ")
      printf(p"tag=0x${Hexadecimal(s1_tag)}, idx=${s1_index}, region_bit=${s1_region_bit}, ")
      printf(p"bitvec: 0x${Hexadecimal(s1_entry_bitvec)} -> 0x${Hexadecimal(s1_entry_bitvec | s1_bit_mask)}\n")
    } .otherwise {
      // 未命中：分配新 entry
      valids(s1_index) := true.B
      entries(s1_index).tag := s1_tag
      entries(s1_index).bitvec := s1_bit_mask
      // Debug: 分配新entry
      printf(p"[BitVecPrefetchFilter] PASS (alloc): addr=0x${Hexadecimal(s1_bits.addr)}, prefetch_type=${s1_type}, ")
      printf(p"tag=0x${Hexadecimal(s1_tag)}, idx=${s1_index}, region_bit=${s1_region_bit}, ")
      printf(p"new bitvec=0x${Hexadecimal(s1_bit_mask)}\n")
    }
  }
  
  // Debug: 过滤情况
  when (s1_should_filter) {
    printf(p"[BitVecPrefetchFilter] FILTER: addr=0x${Hexadecimal(s1_bits.addr)}, prefetch_type=${s1_type}, ")
    printf(p"tag=0x${Hexadecimal(s1_tag)}, idx=${s1_index}, region_bit=${s1_region_bit}, ")
    printf(p"bitvec=0x${Hexadecimal(s1_entry_bitvec)} (bit already set)\n")
  }
  
  // S1 完成后清空
  when (s1_will_complete && !s0_fire) {
    s1_valid := false.B
  }
}


/**
  * PrefetchFilter 工厂对象
  * 用于创建不同类型的过滤器
  */
object PrefetchFilter {
  def apply(filterType: String = "bitvec")(implicit p: Parameters): PrefetchFilter = {
    filterType.toLowerCase match {
      case "null" | "none" | "passthrough" => new NullPrefetchFilter
      case "bitvec" | "region"             => new BitVecPrefetchFilter
      case _ => throw new IllegalArgumentException(s"Unknown prefetch filter type: $filterType")
    }
  }
}
