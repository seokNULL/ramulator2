#ifndef     RAMULATOR_FRONTEND_PROCESSOR_SIMPLEO3_INST_DISPATCHER_H
#define     RAMULATOR_FRONTEND_PROCESSOR_SIMPLEO3_INST_DISPATCHER_H

#include <vector>
#include <list>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <functional>
#include <optional>

#include "base/base.h"
#include "base/clocked.h"
#include "base/debug.h"
#include "base/type.h"
#include "base/request.h"
#include "memory_system/memory_system.h"
#include "addr_mapper/addr_mapper.h"


enum class CuDSchedulingPolicy {
    CuDPriority,
    RoundRobinBank,
    RoundRobinBankGroup,
    RoundRobinRank
};

struct CuDReqType {
    enum : int { RowCopy = 2, Majority3 = 3, Majority5 = 4 };
};

namespace Ramulator {

using RequestQueue = std::list<std::pair<Clk_t, Request>>;

// -------------------------------------------------------
// 32-bit CuD Instruction Bit Layout
// -------------------------------------------------------
//  [31:29]  opcode     (3 bits)
//           001 = RowCopy src
//           010 = RowCopy dst
//           011 = Majority3
//           100 = Majority5
//  [28:22]  reserved   (7 bits)
//  [21]     last       (1 bit, valid only for RowCopy dst)
//  [20:17]  bank addr  (4 bits) → [20:19] bankgroup, [18:17] bank
//  [16:0]   row  addr  (17 bits)
// -------------------------------------------------------
enum class CuDOpcode : uint8_t {
    RowCopySrc = 0b001,
    RowCopyDst = 0b010,
    Majority3  = 0b011,
    Majority5  = 0b100,
    Invalid    = 0xFF,
};

struct CuDInstDecoded {
    CuDOpcode opcode;
    uint32_t  row;       // bits [16:0]
    uint8_t   bankgroup; // bits [20:19]
    uint8_t   bank;      // bits [18:17]
    bool      is_last;   // bit [21], valid only for RowCopy dst
};

class CuDAddrDecoder {
public:
    static CuDInstDecoded decode(uint32_t cud_inst);

    // Decode cud_inst and populate req.addr_vec.
    // addr_vec layout: [channel, rank, bankgroup, bank, row, column]
    // channel / rank / column are not encoded in the instruction → set to 0.
    static void apply(Request& req);

private:
    static constexpr int      kOpcodeShift   = 29;
    static constexpr uint32_t kOpcodeMask    = 0x7U;
    static constexpr int      kLastBit       = 21;
    static constexpr int      kBankShift     = 17;
    static constexpr uint32_t kBankFieldMask = 0xFU;
    static constexpr uint32_t kRowMask       = 0x1FFFFU;
    static constexpr uint32_t kBankGroupMask = 0x3U;
    static constexpr uint32_t kBankMask      = 0x3U;

    static constexpr int kIdxChannel   = 0;
    static constexpr int kIdxRank      = 1;
    static constexpr int kIdxBankGroup = 2;
    static constexpr int kIdxBank      = 3;
    static constexpr int kIdxRow       = 4;
    static constexpr int kIdxColumn    = 5;
    static constexpr int kNumLevels    = 6;
};

class DispatchPolicy {
public:
    virtual ~DispatchPolicy() = default;
    virtual bool select(RequestQueue& cud_queue,
                        RequestQueue& normal_queue,
                        RequestQueue::iterator& out_it,
                        bool& out_is_cud,
                        Clk_t current_clk) = 0;
};

class StrictCuDPriorityPolicy : public DispatchPolicy {
public:
    bool select(RequestQueue& cud_queue,
                RequestQueue& normal_queue,
                RequestQueue::iterator& out_it,
                bool& out_is_cud,
                Clk_t current_clk) override;
};

class RoundRobinPolicy : public DispatchPolicy {
public:
    RoundRobinPolicy(CuDSchedulingPolicy scheduling_policy)
        : m_scheduling_policy(scheduling_policy) {}

    bool select(RequestQueue& cud_queue,
                RequestQueue& normal_queue,
                RequestQueue::iterator& out_it,
                bool& out_is_cud,
                Clk_t current_clk) override;

private:
    bool conflicts(const Request& normal_req) const;

    CuDSchedulingPolicy m_scheduling_policy;
    Clk_t               m_cud_duration  = 48;  //Worst instruction delay (MAJX)

    bool    m_last_was_cud  = false;
    bool    m_cud_active    = false;
    Clk_t   m_cud_start_clk = 0;
    Request m_active_cud;

    static constexpr int kIdxRank      = 1;
    static constexpr int kIdxBankGroup = 2;
    static constexpr int kIdxBank      = 3;
    static constexpr int kIdxRow       = 4;
};

class InstructionDispatcher : public Clocked<InstructionDispatcher> {
public:
    InstructionDispatcher(IMemorySystem* memory_system, CuDSchedulingPolicy scheduling_policy);
    void connect_memory_system(IMemorySystem* ms) { m_memory_system = ms; }
    void init_addr_mapper(IAddrMapper* addr_mapper) { m_addr_mapper = addr_mapper; }
    void set_policy(std::unique_ptr<DispatchPolicy> policy) { m_policy = std::move(policy); }

    bool send_normal(Clk_t clk, Request req);
    bool send_cud   (Clk_t clk, Request req);
    void tick();
    void PrintProfile(Clk_t current_clk) const;

    uint64_t s_normal_dispatched  = 0;
    uint64_t s_cud_dispatched     = 0;
    uint64_t s_cud_completed      = 0;
    uint64_t s_send_failed_cycles = 0;

private:
    void PrintQueue(const RequestQueue& queue, const std::string& name) const;

    IMemorySystem*                  m_memory_system;
    IAddrMapper*                    m_addr_mapper;
    CuDSchedulingPolicy             m_scheduling_policy;
    Logger_t                        m_logger;
    RequestQueue                    m_normal_queue;
    RequestQueue                    m_cud_queue;
    std::unique_ptr<DispatchPolicy> m_policy;

    // static constexpr size_t kNormalDepth = 32;
    // static constexpr size_t kCuDDepth    = 32;
    static constexpr size_t kNormalDepth = 1024;
    static constexpr size_t kCuDDepth    = 1024;
};

} // namespace Ramulator
#endif // RAMULATOR_FRONTEND_PROCESSOR_SIMPLEO3_INST_DISPATCHER_H
