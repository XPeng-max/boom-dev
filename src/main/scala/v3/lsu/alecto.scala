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
}

// ======================= Sandbox Table =========================
class SandboxTableEntry(implicit p: Parameters) extends BoomBundle with HasAlectoParameters {
    val valid = Bool()
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
  })

  io.kill_prefetch := false.B
  // ========== 存储结构 (SyncReadMem) ==========
  val table = SyncReadMem(SANDBOX_TABLE_SIZE, new SandboxTableEntry)

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


  val s1_hit = s1_valid && s1_entry.valid && (s1_entry.addr_tag === s1_addr_tag)
  val s1_is_confirmed = s1_hit && (s1_entry.pc_hash === s1_pc_hash)
  val s1_need_confirmed = s1_is_confirmed && !s1_entry.confirmed


  // ========== 构造新 entry ==========
  val s1_new_entry = Wire(new SandboxTableEntry)
  s1_new_entry.valid := true.B
  s1_new_entry.pc_hash := s1_pc_hash
  s1_new_entry.addr_tag := s1_addr_tag
  s1_new_entry.prefetch_type := s1_prefetch_type - 1.U
  s1_new_entry.confirmed := false.B

  // ========== 写回逻辑 ==========
  val s1_prefetch_valid = s1_valid && s1_is_prefetch
  when (s1_prefetch_valid) {
    // Debug: 预取请求写入sandbox
    printf(p"[SandboxTable] Prefetch recorded:s1_hit=${s1_hit}, addr_tag=0x${Hexadecimal(s1_addr_tag)}, pc_hash=0x${Hexadecimal(s1_pc_hash)}, prefetch_type=${s1_prefetch_type - 1.U}, idx=${s1_idx}\n")
    when (s1_hit) {
      // 不覆盖，丢弃该请求
      io.kill_prefetch := true.B
      bypass_valid := false.B
    }.otherwise {
      // 新请求，分配
      table.write(s1_idx, s1_new_entry)
      // 更新旁路寄存器
      bypass_valid := true.B
      bypass_idx := s1_idx
      bypass_entry := s1_new_entry
    }
  } .otherwise {
    bypass_valid := false.B
    // Debug: 需求请求查询sandbox
    when (s1_hit) {
      when (s1_is_confirmed) {
        s1_new_entry := s1_entry // 保持不变
        s1_new_entry.confirmed := true.B
        table.write(s1_idx, s1_new_entry)
        // 更新旁路寄存器
        bypass_valid := true.B
        bypass_idx := s1_idx
        bypass_entry := s1_new_entry
        printf(p"[SandboxTable] Demand hit CONFIRMED(need_confirmed = ${s1_need_confirmed}): addr_tag=0x${Hexadecimal(s1_addr_tag)}, pc_hash=0x${Hexadecimal(s1_pc_hash)}, prefetch_type=${s1_entry.prefetch_type}, idx=${s1_idx}\n")
      } .otherwise {
        printf(p"[SandboxTable] Demand hit but PC MISMATCH: addr_tag=0x${Hexadecimal(s1_addr_tag)}, req_pc_hash=0x${Hexadecimal(s1_pc_hash)}, entry_pc_hash=0x${Hexadecimal(s1_entry.pc_hash)}, idx=${s1_idx}\n")
      }
    }
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
  assert(!io.req.valid || io.prefetch_type <= 3.U,
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
    val valid = Bool()
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
  require(num_prefetchers > 0 && num_prefetchers <= 3, 
    s"SampleTable: num_prefetchers must be 1-3, got $num_prefetchers")

  val io = IO(new Bundle {
    val sample_update = Flipped(Valid(new SampleUpdate))
    val allocation_update = Decoupled(new AllocationUpdate(num_prefetchers))
  })

  // ========== 存储结构 (SyncReadMem) ==========
  val table = SyncReadMem(SAMPLE_TABLE_SIZE, new SampleTableEntry(num_prefetchers))

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

  // 判断是否命中（valid 且 tag 匹配）
  val s1_hit = s1_read_entry.valid && (s1_read_entry.pc_hash_tag === s1_tag)

  // ========== 构造新 entry ==========
  val s1_new_entry = Wire(new SampleTableEntry(num_prefetchers))
  s1_new_entry := s1_read_entry // 默认保持不变

  val s1_prefetch_valid = s1_valid && s1_is_prefetch
  val s1_demand_valid = s1_valid && !s1_is_prefetch

  when (s1_prefetch_valid) {
    // 预取请求处理
    when (s1_hit) {
      // 命中：更新 issued(prefetch_type) + 1，其他字段保持
      s1_new_entry.valid := true.B
      s1_new_entry.pc_hash_tag := s1_read_entry.pc_hash_tag
      s1_new_entry.demand := s1_read_entry.demand
      s1_new_entry.confirmed := s1_read_entry.confirmed
      s1_new_entry.issued := s1_read_entry.issued
      // 饱和加法
      when (s1_read_entry.issued(s1_prefetch_type) < ((1.U << SAMPLE_TABLE_COUNTER_WIDTH) - 1.U)) {
        s1_new_entry.issued(s1_prefetch_type) := s1_read_entry.issued(s1_prefetch_type) + 1.U
      }
    } .otherwise {
      // 未命中：替换 entry，初始化
      s1_new_entry.valid := true.B
      s1_new_entry.pc_hash_tag := s1_tag
      s1_new_entry.demand := 0.U
      // 初始化所有计数器为 0
      for (i <- 0 until num_prefetchers) {
        s1_new_entry.issued(i) := 0.U
        s1_new_entry.confirmed(i) := 0.U
      }
      s1_new_entry.issued(s1_prefetch_type) := 1.U
    }
    printf(p"[SampleTable] Issued updated for prefetch_type ${s1_prefetch_type}, pc_hash: ${s1_pc_hash}, confirmed: ${s1_new_entry.confirmed(s1_prefetch_type)}, issued: ${s1_new_entry.issued(s1_prefetch_type)}\n")
  } .elsewhen (s1_demand_valid && s1_hit) {
    // 命中：更新 demand + 1
    s1_new_entry.valid := true.B
    s1_new_entry.pc_hash_tag := s1_read_entry.pc_hash_tag
    s1_new_entry.issued := s1_read_entry.issued
    s1_new_entry.confirmed := s1_read_entry.confirmed
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
    // 如果 is_confirmed，更新 confirmed(prefetch_type)
    when (s1_is_confirmed) {
      when (s1_read_entry.confirmed(s1_prefetch_type) < ((1.U << SAMPLE_TABLE_COUNTER_WIDTH) - 1.U)) {
        s1_new_entry.confirmed(s1_prefetch_type) := s1_read_entry.confirmed(s1_prefetch_type) + 1.U
      }
      printf(p"[SampleTable] Confirmed updated for prefetch_type ${s1_prefetch_type}, pc_hash: ${s1_pc_hash}, confirmed: ${s1_new_entry.confirmed(s1_prefetch_type)}, issued: ${s1_new_entry.issued(s1_prefetch_type)}\n")
    }
  }

  // ========== 写回逻辑 ==========
  // 只有在有效更新时才写回：
  // - 预取请求总是写回（命中更新或未命中替换）
  // - 需求请求只在命中时写回
  val s1_should_write = s1_valid && (s1_is_prefetch || s1_hit)

  when (s1_should_write) {
    table.write(s1_idx, s1_new_entry)
    // 更新旁路寄存器
    bypass_valid := true.B
    bypass_idx := s1_idx
    bypass_entry := s1_new_entry
  } .otherwise {
    bypass_valid := false.B
  }

  // ========== Allocation 输出逻辑 ==========
  // 当 demand >= SAMPLE_DEMAND_THRESHOLD 时，输出 allocation_update
  // 使用更新后的 demand 值进行判断
  val s1_demand_threshold_reached = s1_valid && s1_hit && !s1_is_prefetch && 
                                     (s1_new_entry.demand >= SAMPLE_DEMAND_THRESHOLD.U)

  // 使用寄存器保存 allocation 输出，直到被接收
  val alloc_valid = RegInit(false.B)
  val alloc_bits = Reg(new AllocationUpdate(num_prefetchers))

  when (s1_demand_threshold_reached && (!alloc_valid || io.allocation_update.fire)) {
    // 触发新的 allocation 输出
    alloc_valid := true.B
    alloc_bits.pc_hash := s1_pc_hash
    alloc_bits.issued := s1_new_entry.issued
    alloc_bits.confirmed := s1_new_entry.confirmed
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
    val valid = Bool() // 有效位
    val pc_hash_tag = UInt(SAMPLE_TABLE_TAG_BITS.W) // PC标签部分
    val prefetch_degree = Vec(num_prefetchers, UInt(ALLOCATION_DEGREE_WIDTH.W)) // 对于各个预取器的预取度
}

class AllocationRequest(implicit p: Parameters) extends BoomBundle with HasAlectoParameters {
    val pc = UInt(coreMaxAddrBits.W)
}

class AllocationResponseBundle(num_prefetchers: Int)(implicit p: Parameters) extends BoomBundle with HasAlectoParameters {
    val prefetch_degree = Vec(num_prefetchers, UInt(ALLOCATION_DEGREE_WIDTH.W))
}


class AllocationTable(num_prefetchers: Int)(implicit p: Parameters) extends BoomModule with HasAlectoParameters {
  require(num_prefetchers > 0 && num_prefetchers <= 3, 
    s"AllocationTable: num_prefetchers must be 1-3, got $num_prefetchers")

  val io = IO(new Bundle {
    val allocation_update = Flipped(Decoupled(new AllocationUpdate(num_prefetchers)))
    val allocation_req = Flipped(Valid(new AllocationRequest))
    val allocation_resp = Valid(new AllocationResponseBundle(num_prefetchers))
  })
  val table = SyncReadMem(ALLOCATION_TABLE_SIZE, new AllocationTableEntry(num_prefetchers))

  // ========== 地址解析辅助函数 ==========
  // BUGFIX: These functions should use the parameter, not the captured s0_pc_hash
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

  // 判断是否命中
  val s1_req_hit = s1_req_read_entry.valid && (s1_req_read_entry.pc_hash_tag === s1_req_tag)

  // 构造响应
  val s1_req_resp = Wire(new AllocationResponseBundle(num_prefetchers))
  when (s1_req_hit) {
    s1_req_resp.prefetch_degree := s1_req_read_entry.prefetch_degree
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
        printf(p"[${i.U}]=${s1_req_read_entry.prefetch_degree(i)} ")
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

  // 判断是否命中
  val s1_update_hit = s1_update_read_entry.valid && (s1_update_read_entry.pc_hash_tag === s1_update_tag)

  // 计算 is_promote 和 is_demote
  val is_promote = VecInit((0 until num_prefetchers).map { i =>
    (s1_update_confirmed(i) >= ((s1_update_issued(i) >> 1) + (s1_update_issued(i) >> 2))) && (s1_update_issued(i) =/= 0.U)
  })
  val is_demote = VecInit((0 until num_prefetchers).map { i =>
    (s1_update_confirmed(i) <= (s1_update_issued(i) >> 2)) && (s1_update_issued(i) =/= 0.U)
  })

  // 构造新 entry
  val s1_new_entry = Wire(new AllocationTableEntry(num_prefetchers))
  s1_new_entry.valid := true.B
  s1_new_entry.pc_hash_tag := Mux(s1_update_hit, s1_update_read_entry.pc_hash_tag, s1_update_tag)
  
  val max_degree = (1.U << ALLOCATION_DEGREE_WIDTH) - 1.U
  for (i <- 0 until num_prefetchers) {
    val base_degree = Mux(s1_update_hit, s1_update_read_entry.prefetch_degree(i), 1.U)
    s1_new_entry.prefetch_degree(i) := MuxCase(base_degree, Seq(
      is_promote(i) -> Mux(base_degree < max_degree, base_degree + 1.U, base_degree),
      is_demote(i)  -> Mux(base_degree > 0.U, base_degree - 1.U, base_degree)
    ))
  }

  // ========== 写回逻辑 ==========
  val s1_should_write = s1_update_valid

  when (s1_should_write) {
    table.write(s1_update_idx, s1_new_entry)
    // 更新旁路寄存器
    bypass_valid := true.B
    bypass_idx := s1_update_idx
    bypass_entry := s1_new_entry
    
    // Debug: allocation_update 更新结果
    printf(p"[AllocationTable] Update: pc_hash=0x${Hexadecimal(s1_update_pc_hash)}, idx=${s1_update_idx}, hit=${s1_update_hit}\n")
    for (i <- 0 until num_prefetchers) {
      val old_degree = Mux(s1_update_hit, s1_update_read_entry.prefetch_degree(i), 1.U)
      printf(p"  [prefetcher ${i.U}] issued=${s1_update_issued(i)}, confirmed=${s1_update_confirmed(i)}, ")
      printf(p"promote=${is_promote(i)}, demote=${is_demote(i)}, ")
      printf(p"degree: ${old_degree} -> ${s1_new_entry.prefetch_degree(i)}\n")
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