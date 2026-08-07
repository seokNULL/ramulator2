#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include "frontend/impl/processor/simpleO3/inst_dispatcher.h"

// #define LOG_INST_DISPATCH
namespace Ramulator {

// -------------------------------------------------------
// StrictCuDPriorityPolicy
// -------------------------------------------------------
bool StrictCuDPriorityPolicy::select(RequestQueue& cud_queue,
                                      RequestQueue& normal_queue,
                                      RequestQueue::iterator& out_it,
                                      bool& out_is_cud,
                                      Clk_t /*current_clk*/) {
    if (!cud_queue.empty()) {
        out_it = cud_queue.begin(); out_is_cud = true;  return true;
    }
    if (!normal_queue.empty()) {
        out_it = normal_queue.begin(); out_is_cud = false; return true;
    }
    return false;
}

// -------------------------------------------------------
// CuDAddrDecoder
// -------------------------------------------------------
CuDInstDecoded CuDAddrDecoder::decode(uint32_t inst) {
    CuDInstDecoded d;

    uint8_t op = static_cast<uint8_t>((inst >> kOpcodeShift) & kOpcodeMask);
    switch (op) {
        case 0b001: d.opcode = CuDOpcode::RowCopySrc; break;
        case 0b010: d.opcode = CuDOpcode::RowCopyDst; break;
        case 0b011: d.opcode = CuDOpcode::Majority3;  break;
        case 0b100: d.opcode = CuDOpcode::Majority5;  break;
        default:    d.opcode = CuDOpcode::Invalid;     break;
    }
    d.row = inst & kRowMask;

    uint8_t bank_field = static_cast<uint8_t>((inst >> kBankShift) & kBankFieldMask);
    d.bankgroup = (bank_field >> 2) & kBankGroupMask;
    d.bank      =  bank_field       & kBankMask;

    d.is_last = static_cast<bool>((inst >> kLastBit) & 0x1U);

    return d;
}

void CuDAddrDecoder::apply(Request& req) {
    if (static_cast<int>(req.addr_vec.size()) < kNumLevels)
        req.addr_vec.assign(kNumLevels, 0);

    CuDInstDecoded d = decode(req.cud_inst);
    req.addr_vec[kIdxChannel]   = 0;
    req.addr_vec[kIdxRank]      = 0;
    req.addr_vec[kIdxBankGroup] = d.bankgroup;
    req.addr_vec[kIdxBank]      = d.bank;
    req.addr_vec[kIdxRow]       = static_cast<int>(d.row);
    req.addr_vec[kIdxColumn]    = 0;
}

// -------------------------------------------------------
// RoundRobinPolicy
// -------------------------------------------------------
bool RoundRobinPolicy::conflicts(const Request& normal_req) const {
    if (!m_cud_active) return false;
    const auto& cv = m_active_cud.addr_vec;
    const auto& nv = normal_req.addr_vec;
    switch (m_scheduling_policy) {
        case CuDSchedulingPolicy::RoundRobinRank:
            return (cv[kIdxRank] == nv[kIdxRank]);
        case CuDSchedulingPolicy::RoundRobinBankGroup:
            return (cv[kIdxRank]      == nv[kIdxRank]) &&
                   (cv[kIdxBankGroup] == nv[kIdxBankGroup]);
        case CuDSchedulingPolicy::RoundRobinBank:
            return (cv[kIdxRank]      == nv[kIdxRank]) &&
                   (cv[kIdxBankGroup] == nv[kIdxBankGroup]) &&
                   (cv[kIdxBank]      == nv[kIdxBank]);
        default:
            return false;
    }
}

bool RoundRobinPolicy::select(RequestQueue& cud_queue,
                               RequestQueue& normal_queue,
                               RequestQueue::iterator& out_it,
                               bool& out_is_cud,
                               Clk_t current_clk) {
    // Most recently issued CuD inst's duration check
    if (m_cud_active && (current_clk - m_cud_start_clk >= m_cud_duration))
        m_cud_active = false;

    // Determine round-ronin turn
    if (cud_queue.empty() && normal_queue.empty())
        return false;
    bool cud_turn    = !m_last_was_cud;
    bool normal_turn = !cud_turn;
    if (cud_turn    && cud_queue.empty())    { cud_turn = false; normal_turn = true;  }
    if (normal_turn && normal_queue.empty()) { cud_turn = true;  normal_turn = false; }

    // Check conflicts
    if (normal_turn && conflicts(normal_queue.front().second)) {
        const auto& nv = normal_queue.front().second.addr_vec;
        const auto& cv = m_active_cud.addr_vec;
        #ifdef LOG_INST_DISPATCH
        printf("[RoundRobin] Normal BLOCKED by active CuD (policy=%d) clk:%ld\n"
               "             Normal  -> rank:%d  bg:%d  bank:%d\n"
               "             CuD     -> rank:%d  bg:%d  bank:%d  row:%d\n",
               static_cast<int>(m_scheduling_policy), current_clk,
               nv[kIdxRank], nv[kIdxBankGroup], nv[kIdxBank],
               cv[kIdxRank], cv[kIdxBankGroup], cv[kIdxBank], cv[kIdxRow]);
        #endif
        return false;
    }

    if (cud_turn) {
        out_it     = cud_queue.begin();
        out_is_cud = true;
        if (!m_cud_active || m_active_cud.cud_inst != cud_queue.front().second.cud_inst) {
            m_active_cud    = cud_queue.front().second;
            m_cud_active    = true;
            m_cud_start_clk = current_clk;
        }
        m_last_was_cud = true;
    } else {
        out_it         = normal_queue.begin();
        out_is_cud     = false;
        m_last_was_cud = false;
    }

    return true;
}


// -------------------------------------------------------
// InstructionDispatcher
// -------------------------------------------------------
InstructionDispatcher::InstructionDispatcher(IMemorySystem* ms,
                                             CuDSchedulingPolicy scheduling_policy)
    : m_memory_system(ms), m_scheduling_policy(scheduling_policy) {
    m_logger = Logging::create_logger("InstructionDispatcher");
    switch (scheduling_policy) {
        case CuDSchedulingPolicy::CuDPriority:
            m_policy = std::make_unique<StrictCuDPriorityPolicy>();
            break;
        case CuDSchedulingPolicy::RoundRobinBank:
        case CuDSchedulingPolicy::RoundRobinBankGroup:
        case CuDSchedulingPolicy::RoundRobinRank:
            m_policy = std::make_unique<RoundRobinPolicy>(scheduling_policy);
            break;
    }
}

bool InstructionDispatcher::send_normal(Clk_t clk, Request req) {
    if (m_normal_queue.size() >= kNormalDepth) {
        printf("[Dispatcher] Normal queue FULL. Reject addr:%ld at clk:%ld\n", req.addr, clk);
        return false;
    }
    m_addr_mapper->apply(req);
    m_normal_queue.push_back(std::make_pair(clk, req));
    #ifdef LOG_INST_DISPATCH
    printf("[Dispatcher] Enqueue NORMAL  addr:%-16ld enq_clk:%ld\n", req.addr, clk);
    #endif
    return true;
}

bool InstructionDispatcher::send_cud(Clk_t clk, Request req) {
    if (m_cud_queue.size() >= kCuDDepth) {
        printf("[Dispatcher] CuD queue FULL. Reject inst:%08x at clk:%ld\n", req.cud_inst, clk);
        return false;
    }
    CuDAddrDecoder::apply(req);
    m_cud_queue.push_back(std::make_pair(clk, req));
    const CuDInstDecoded d = CuDAddrDecoder::decode(req.cud_inst);
    #ifdef LOG_INST_DISPATCH
    printf("[Dispatcher] Enqueue CuD     inst:%-8x  bg:%u  bank:%u  row:%-6u  enq_clk:%ld\n",
           req.cud_inst, d.bankgroup, d.bank, d.row, clk);
    #endif
    return true;
}

void InstructionDispatcher::tick() {
    m_clk++;
    if (!m_addr_mapper)
        m_addr_mapper = m_memory_system->get_ifce<IAddrMapper>();

    RequestQueue::iterator selected_it;
    bool is_cud = false;
    if (!m_policy->select(m_cud_queue, m_normal_queue, selected_it, is_cud, m_clk))
        return;

    if (m_memory_system->send(selected_it->second)) {
        if (is_cud) {
            s_cud_dispatched++;
            const CuDInstDecoded d = CuDAddrDecoder::decode(selected_it->second.cud_inst);
            #ifdef LOG_INST_DISPATCH
            printf("[Dispatcher] Dispatched CuD   inst:%-8x  bg:%u  bank:%u  row:%-6u  clk:%ld\n",
                   selected_it->second.cud_inst, d.bankgroup, d.bank, d.row, m_clk);
            #endif
            m_cud_queue.erase(selected_it);
        } else {
            s_normal_dispatched++;
            #ifdef LOG_INST_DISPATCH
            printf("[Dispatcher] Dispatched NORMAL addr:%-16ld clk:%ld\n",
                   selected_it->second.addr, m_clk);
            #endif
            m_normal_queue.erase(selected_it);
        }
    } else {
        s_send_failed_cycles++;
        if (is_cud)
            printf("[Dispatcher] send() FAILED (memory system busy) CuD   inst:%08x\n",
                   selected_it->second.cud_inst);
        else
            printf("[Dispatcher] send() FAILED (memory system busy) NORMAL addr:%ld\n",
                   selected_it->second.addr);
    }
}

void InstructionDispatcher::PrintQueue(const RequestQueue& queue,
                                       const std::string& name) const {
    printf("  [%s] entries:%zu\n", name.c_str(), queue.size());
    if (queue.empty()) { printf("    (empty)\n"); return; }
    printf("    %-6s  %-10s  %-5s  %-5s  %-8s  %-10s\n",
           "idx", "inst/addr", "bg", "bank", "row", "enq_clk");
    printf("    %-6s  %-10s  %-5s  %-5s  %-8s  %-10s\n",
           "---", "---------", "--", "----", "---", "-------");
    int idx = 0;
    for (const auto& entry : queue) {
        const Clk_t    enq_clk = entry.first;
        const Request& req     = entry.second;
        if (req.type_id == Request::Type::CuD) {
            const CuDInstDecoded d = CuDAddrDecoder::decode(req.cud_inst);
            printf("    %-6d  %08x    %-5u  %-5u  %-8u  %-10ld\n",
                   idx, req.cud_inst, d.bankgroup, d.bank, d.row, enq_clk);
        } else {
            printf("    %-6d  addr:%-13ld  %-5s  %-5s  %-8s  %-10ld\n",
                   idx, req.addr, "-", "-", "-", enq_clk);
        }
        idx++;
    }
}

void InstructionDispatcher::PrintProfile(Clk_t current_clk) const {
    printf("\n");
    printf("+------------------------------------------------------+\n");
    printf("|      InstructionDispatcher Profile                   |\n");
    printf("+------------------------------------------------------+\n");
    printf("|  [Statistics]                                        |\n");
    printf("|  CuD  dispatched       : %-26lu|\n", s_cud_dispatched);
    printf("|  CuD  completed        : %-26lu|\n", s_cud_completed);
    printf("|  Normal dispatched     : %-26lu|\n", s_normal_dispatched);
    printf("|  Send failed cycles    : %-26lu|\n", s_send_failed_cycles);
    printf("+------------------------------------------------------+\n");
    printf("\n");
}

} // namespace Ramulator
