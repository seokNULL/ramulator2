#include <filesystem>
#include <iostream>
#include <fstream>

#include <spdlog/spdlog.h>

#include "base/exception.h"
#include "base/utils.h"
#include "frontend/impl/processor/simpleO3/core.h"
#include "frontend/impl/processor/simpleO3/llc.h"



namespace Ramulator {

namespace fs = std::filesystem;

SimpleO3Core::Trace::Trace(std::string file_path_str) {
  fs::path trace_path(file_path_str);
  if (!fs::exists(trace_path)) {
    throw ConfigurationError("Trace {} does not exist!", file_path_str);
  }

  std::ifstream trace_file(trace_path);
  if (!trace_file.is_open()) {
    throw ConfigurationError("Trace {} cannot be opened!", file_path_str);
  }

  std::string line;
  while (std::getline(trace_file, line)) {
    std::vector<std::string> tokens;
    tokenize(tokens, line, " ");

    int num_tokens = tokens.size();
    int bubble_count = std::stoi(tokens[0]);
    Addr_t load_addr = std::stoll(tokens[1]);

    if(num_tokens != 2 & num_tokens !=3 & num_tokens !=4){
      throw ConfigurationError("Trace {} format invailid", file_path_str);
    }

    if(num_tokens == 4){
      // CuD_inst_t cud_inst = std::stoi(tokens[2], 0, 16);
      CuD_inst_t cud_inst = std::stoul(tokens[2], 0, 16);
      m_trace.push_back({bubble_count, -1, -1, true, cud_inst});
    }
    else if (num_tokens == 3) {
      Addr_t store_addr = std::stoll(tokens[2]);
      m_trace.push_back({bubble_count, load_addr, store_addr, false, -1});
    }
    else if (num_tokens == 2) {
      m_trace.push_back({bubble_count, load_addr, -1, false, -1});
    }
  }
  trace_file.close();
  m_trace_length = m_trace.size();
}

const SimpleO3Core::Trace::Inst& SimpleO3Core::Trace::get_next_inst() {
  const Inst& inst = m_trace[m_curr_trace_idx];
  m_curr_trace_idx = (m_curr_trace_idx + 1) % m_trace_length;
  // m_curr_trace_idx = (m_curr_trace_idx + 1);
  return inst;
}


SimpleO3Core::InstWindow::InstWindow(int ipc, int depth):
m_ipc(ipc), m_depth(depth),
m_ready_list(depth, false), m_addr_list(depth, -1) {};

bool SimpleO3Core::InstWindow::is_full() {
  return m_load == m_depth;
}

void SimpleO3Core::InstWindow::insert(bool ready, Addr_t addr) {
  m_ready_list.at(m_head_idx) = ready;
  m_addr_list.at(m_head_idx) = addr;

  m_head_idx = (m_head_idx + 1) % m_depth;
  m_load++;
}

int SimpleO3Core::InstWindow::retire() {
  if (m_load == 0) return 0;

  int num_retired = 0;
  while (m_load > 0 && num_retired < m_ipc) {
    if (!m_ready_list.at(m_tail_idx))
      break;

    m_tail_idx = (m_tail_idx + 1) % m_depth;
    m_load--;
    num_retired++;
  }
  return num_retired;
}

void SimpleO3Core::InstWindow::set_ready(Addr_t addr) {
  if (m_load == 0) return;

  int index = m_tail_idx;
  for (int i = 0; i < m_load; i++) {
    if (m_addr_list[index] == addr) {
      m_ready_list[index] = true;
    }
    index++;
    if (index == m_depth) {
      index = 0;
    }
  }
}

// SimpleO3Core::SimpleO3Core(int id, int ipc, int depth, size_t num_expected_insts, std::string trace_path, ITranslation* translation, SimpleO3LLC* llc):
// m_id(id), m_window(ipc, depth), m_trace(trace_path), m_num_expected_insts(num_expected_insts), m_translation(translation), m_llc(llc) {
SimpleO3Core::SimpleO3Core(int id, int ipc, int depth, size_t num_expected_insts, std::string trace_path, ITranslation* translation, SimpleO3LLC* llc, InstructionDispatcher* dispatcher):
m_id(id), m_window(ipc, depth), m_trace(trace_path), m_num_expected_insts(num_expected_insts), m_translation(translation), m_llc(llc), m_dispatcher(dispatcher) {
  // Fetch the instructions and addresses for tick 0
  auto inst = m_trace.get_next_inst();
  m_num_bubbles = inst.bubble_count;
  m_load_addr = inst.load_addr;
  m_writeback_addr = inst.store_addr;
  m_cud_pending = inst.is_cud;
  m_cud_inst = inst.cud_inst;
}

void SimpleO3Core::tick() {
  m_clk++;
  // if(m_trace.m_trace_length != m_num_expected_insts) {
  //   throw std::runtime_error("Expected instruction num and trace length mismatch!");
  // }

  s_insts_retired += m_window.retire();
  if (!reached_expected_num_insts) {
    if (s_insts_retired >= m_num_expected_insts) {
      reached_expected_num_insts = true;
      s_cycles_recorded = m_clk;
    }
  }

  // if(m_trace.m_trace_length < m_trace.m_curr_trace_idx) {
  //   return;
  // }

  // First, issue the non-memory instructions
  int num_inserted_insts = 0;
  while (m_num_bubbles > 0) {
    if (num_inserted_insts == m_window.m_ipc) {
      return;
    }
    if (m_window.is_full()) {
      return;
    };
    m_window.insert(true, -1);
    num_inserted_insts++;
    m_num_bubbles--;
  }

  // CuD instruction interface
  if (m_cud_pending & !m_cud_done) {
    Request cud_request(m_cud_inst, Request::Type::CuD);
    if (cud_request.cud_inst == 0x0000FFFF) {
      m_cud_done = true;
      return;
    }
    if(m_dispatcher->send_cud(m_clk, cud_request)) {
      s_cud_insts_retired++;
    }
    else {
      printf("send_cud to inst_dispatcher failed\n");
      return;
    }
  }

  // Second, try to send the load to the LLC
  if (m_load_addr != -1) {
    if (num_inserted_insts == m_window.m_ipc) {
      return;
    }
    if (m_window.is_full()) {
      return;
    };

    Request load_request(m_load_addr, Request::Type::Read, m_id, m_callback);
    if (!m_translation->translate(load_request)) {
      return;
    };

    if (m_llc->send(load_request)) {
      m_window.insert(false, load_request.addr);
      m_load_addr = -1;
      if (m_writeback_addr != -1) {
        // If there is still writeback, return without getting the next trace line
        // The write back will be issued in the next cycle
        // TODO: Should we allow both load and writeback to issue at the same cycle?
        return;
      }
    } else {
      return;
    }
  }

  // Third, try to send the writeback to the LLC
  if (m_writeback_addr != -1) {
    Request writeback_request(m_writeback_addr, Request::Type::Write, m_id, m_callback);
    if (!m_translation->translate(writeback_request)) {
      return;
    };
    if (!m_llc->send(writeback_request)) {
      return;
    }
  }

  auto inst = m_trace.get_next_inst();
  m_num_bubbles = inst.bubble_count;
  m_load_addr = inst.load_addr;
  m_writeback_addr = inst.store_addr;
  m_cud_pending = inst.is_cud;
  m_cud_inst = inst.cud_inst;
}

void SimpleO3Core::receive(Request& req) {
  m_window.set_ready(req.addr);

  if (req.arrive != -1 && req.depart > m_last_mem_cycle) {
    if (!reached_expected_num_insts) {
      s_mem_access_cycles += (req.depart - std::max(m_last_mem_cycle, req.arrive));
      m_last_mem_cycle = req.depart;
    }
  }
}

}        // namespace Ramulator
