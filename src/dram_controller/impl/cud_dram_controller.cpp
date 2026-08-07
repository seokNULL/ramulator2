#include <queue>

#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"
#include "frontend/impl/processor/simpleO3/inst_dispatcher.h"

// —————————————————––
// Debug print control
// Uncomment to enable debug prints
// —————————————————––
// #define DEBUG_CUD_VERBOSE     // CuD command issue / stall logs
// #define DEBUG_CUD_BUFFER      // Buffer size change logs

#ifdef DEBUG_CUD_VERBOSE
#define CUD_LOG(fmt, ...) printf("[CuD Verbose] " fmt, ##__VA_ARGS__)
#else
#define CUD_LOG(fmt, ...)  do {} while(0)
#endif

#ifdef DEBUG_CUD_BUFFER
#define BUF_LOG(fmt, ...) printf("[CuD Buffer] " fmt, ##__VA_ARGS__)
#else
#define BUF_LOG(fmt, ...)  do {} while(0)
#endif

namespace Ramulator {

class CuDDRAMController final : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAMController, CuDDRAMController, "CuD", "A CuD DRAM controller.");
  private:
    std::deque<Request> pending;          // A queue for read requests that are about to finish (callback after RL)

    ReqBuffer m_active_buffer;            // Buffer for requests being served. This has the highest priority
    ReqBuffer m_priority_buffer;          // Buffer for high-priority requests (e.g., maintenance like refresh).
    ReqBuffer m_read_buffer;              // Read request buffer
    ReqBuffer m_write_buffer;             // Write request buffer
    ReqBuffer m_cud_buffer;

    int m_bank_addr_idx = -1;
    int m_cact_cmd_id   = -1;  // Command ID of CACT on the current DRAM device

    float m_wr_low_watermark;
    float m_wr_high_watermark;
    bool  m_is_write_mode = false;

    size_t s_row_hits = 0;
    size_t s_row_misses = 0;
    size_t s_row_conflicts = 0;
    size_t s_read_row_hits = 0;
    size_t s_read_row_misses = 0;
    size_t s_read_row_conflicts = 0;
    size_t s_write_row_hits = 0;
    size_t s_write_row_misses = 0;
    size_t s_write_row_conflicts = 0;

    size_t m_num_cores = 0;
    std::vector<size_t> s_read_row_hits_per_core;
    std::vector<size_t> s_read_row_misses_per_core;
    std::vector<size_t> s_read_row_conflicts_per_core;

    size_t s_num_read_reqs = 0;
    size_t s_num_write_reqs = 0;
    size_t s_num_other_reqs = 0;
    size_t s_queue_len = 0;
    size_t s_read_queue_len = 0;
    size_t s_write_queue_len = 0;
    size_t s_priority_queue_len = 0;
    float s_queue_len_avg = 0;
    float s_read_queue_len_avg = 0;
    float s_write_queue_len_avg = 0;
    float s_priority_queue_len_avg = 0;

    size_t s_read_latency = 0;
    float s_avg_read_latency = 0;
  //SYKIM
    size_t m_prev_read_size = 0;
    size_t m_prev_write_size = 0;
    size_t m_prev_active_size = 0;
    size_t m_prev_priority_size = 0;
    size_t m_prev_cud_size = 0;
    bool m_is_buff_initialized = false;

    struct CuDCmd
    {
      int       cmd_id;
      AddrVec_t addr_vec;
    };
    std::queue<CuDCmd> m_cud_cmd_queue;
    Request* m_cud_inflight = nullptr;


  // -------------------------------------------------------
  // CuD profiling statistics
  // -------------------------------------------------------
  // Per-command issue counts
  size_t s_cud_act_count_rowcopy  = 0;
  size_t s_cud_act_count_maj3  = 0;
  size_t s_cud_act_count_maj5  = 0;

  size_t s_cud_pre_count  = 0;
  size_t s_cud_completed  = 0;    // Number of fully completed CuD instructions

  // Per-opcode issue counts
  size_t s_cud_rowcopy_src_count = 0;
  size_t s_cud_rowcopy_dst_count = 0;
  size_t s_cud_majority3_count   = 0;
  size_t s_cud_majority5_count   = 0;

  // Execution time tracking
  Clk_t  m_cud_inst_start_clk    = 0;    // clk when current CuD inst started
  size_t s_cud_total_exec_cycles  = 0;    // total cycles spent executing CuD insts
  size_t s_cud_stall_cycles       = 0;    // cycles where CuD cmd was not ready
  size_t s_cud_refresh_pending_cycles = 0; // cycles where refresh was blocked by CuD

  // Per-opcode total execution cycles
  size_t s_cud_rowcopy_src_exec_cycles = 0;
  size_t s_cud_rowcopy_dst_exec_cycles = 0;
  size_t s_cud_majority3_exec_cycles   = 0;
  size_t s_cud_majority5_exec_cycles   = 0;

  CuDOpcode m_cud_inflight_opcode = CuDOpcode::Invalid; // opcode of current inflight inst


  public:
    void init() override {
      m_wr_low_watermark =  param<float>("wr_low_watermark").desc("Threshold for switching back to read mode.").default_val(0.2f);
      m_wr_high_watermark = param<float>("wr_high_watermark").desc("Threshold for switching to write mode.").default_val(0.8f);

      m_scheduler = create_child_ifce<IScheduler>();
      m_refresh = create_child_ifce<IRefreshManager>();
      m_rowpolicy = create_child_ifce<IRowPolicy>();

      if (m_config["plugins"]) {
        YAML::Node plugin_configs = m_config["plugins"];
        for (YAML::iterator it = plugin_configs.begin(); it != plugin_configs.end(); ++it) {
          m_plugins.push_back(create_child_ifce<IControllerPlugin>(*it));
        }
      }
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = memory_system->get_ifce<IDRAM>();
      m_logger = Logging::create_logger("CuDDRAMController[" + std::to_string(m_channel_id) + "]");
      m_bank_addr_idx = m_dram->m_levels("bank");

      m_cact_cmd_id = m_dram->get_cact_cmd_id();
      if (m_cact_cmd_id == -1) {
        throw std::runtime_error(
          "CuDDRAMController: DRAM type does not support CuD (no CACT command). "
          "Add CACT to the DRAM implementation and override get_cact_cmd_id()."
        );
      }
      m_priority_buffer.max_size = 512*3 + 32;

      m_num_cores = frontend->get_num_cores();

      s_read_row_hits_per_core.resize(m_num_cores, 0);
      s_read_row_misses_per_core.resize(m_num_cores, 0);
      s_read_row_conflicts_per_core.resize(m_num_cores, 0);

      register_stat(s_row_hits).name("row_hits_{}", m_channel_id);
      register_stat(s_row_misses).name("row_misses_{}", m_channel_id);
      register_stat(s_row_conflicts).name("row_conflicts_{}", m_channel_id);
      register_stat(s_read_row_hits).name("read_row_hits_{}", m_channel_id);
      register_stat(s_read_row_misses).name("read_row_misses_{}", m_channel_id);
      register_stat(s_read_row_conflicts).name("read_row_conflicts_{}", m_channel_id);
      register_stat(s_write_row_hits).name("write_row_hits_{}", m_channel_id);
      register_stat(s_write_row_misses).name("write_row_misses_{}", m_channel_id);
      register_stat(s_write_row_conflicts).name("write_row_conflicts_{}", m_channel_id);

      for (size_t core_id = 0; core_id < m_num_cores; core_id++) {
        register_stat(s_read_row_hits_per_core[core_id]).name("read_row_hits_core_{}", core_id);
        register_stat(s_read_row_misses_per_core[core_id]).name("read_row_misses_core_{}", core_id);
        register_stat(s_read_row_conflicts_per_core[core_id]).name("read_row_conflicts_core_{}", core_id);
      }

      register_stat(s_num_read_reqs).name("num_read_reqs_{}", m_channel_id);
      register_stat(s_num_write_reqs).name("num_write_reqs_{}", m_channel_id);
      register_stat(s_num_other_reqs).name("num_other_reqs_{}", m_channel_id);
      register_stat(s_queue_len).name("queue_len_{}", m_channel_id);
      register_stat(s_read_queue_len).name("read_queue_len_{}", m_channel_id);
      register_stat(s_write_queue_len).name("write_queue_len_{}", m_channel_id);
      register_stat(s_priority_queue_len).name("priority_queue_len_{}", m_channel_id);
      register_stat(s_queue_len_avg).name("queue_len_avg_{}", m_channel_id);
      register_stat(s_read_queue_len_avg).name("read_queue_len_avg_{}", m_channel_id);
      register_stat(s_write_queue_len_avg).name("write_queue_len_avg_{}", m_channel_id);
      register_stat(s_priority_queue_len_avg).name("priority_queue_len_avg_{}", m_channel_id);

      register_stat(s_read_latency).name("read_latency_{}", m_channel_id);
      register_stat(s_avg_read_latency).name("avg_read_latency_{}", m_channel_id);

      // CuD profiling stats
      register_stat(s_cud_act_count_rowcopy).name("cud_act_count_rowcopy{}", m_channel_id);
      register_stat(s_cud_act_count_maj3).name("cud_act_count_maj3{}", m_channel_id);
      register_stat(s_cud_act_count_maj5).name("cud_act_count_maj5{}", m_channel_id);

      register_stat(s_cud_pre_count).name("cud_pre_count_{}", m_channel_id);
      register_stat(s_cud_completed).name("cud_completed_{}", m_channel_id);
      // register_stat(s_cud_rowcopy_src_count).name("cud_rowcopy_src_count_{}", m_channel_id);
      // register_stat(s_cud_rowcopy_dst_count).name("cud_rowcopy_dst_count_{}", m_channel_id);
      // register_stat(s_cud_majority3_count).name("cud_majority3_count_{}", m_channel_id);
      // register_stat(s_cud_majority5_count).name("cud_majority5_count_{}", m_channel_id);
      // register_stat(s_cud_total_exec_cycles).name("cud_total_exec_cycles_{}", m_channel_id);
      // register_stat(s_cud_stall_cycles).name("cud_stall_cycles_{}", m_channel_id);
      // register_stat(s_cud_refresh_pending_cycles).name("cud_refresh_pending_cycles_{}", m_channel_id);
      // register_stat(s_cud_rowcopy_src_exec_cycles).name("cud_rowcopy_src_exec_cycles_{}", m_channel_id);
      // register_stat(s_cud_rowcopy_dst_exec_cycles).name("cud_rowcopy_dst_exec_cycles_{}", m_channel_id);
      // register_stat(s_cud_majority3_exec_cycles).name("cud_majority3_exec_cycles_{}", m_channel_id);
      // register_stat(s_cud_majority5_exec_cycles).name("cud_majority5_exec_cycles_{}", m_channel_id);
    };

    bool send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      switch (req.type_id) {
        case Request::Type::Read: {
          s_num_read_reqs++;
          break;
        }
        case Request::Type::Write: {
          s_num_write_reqs++;
          break;
        }
        case Request::Type::CuD: {
          s_num_other_reqs++;
          break;
        }
        default: {
          s_num_other_reqs++;
          break;
        }
      }

      // Forward existing write requests to incoming read requests
      if (req.type_id == Request::Type::Read) {
        auto compare_addr = [req](const Request& wreq) {
          return wreq.addr == req.addr;
        };
        if (std::find_if(m_write_buffer.begin(), m_write_buffer.end(), compare_addr) != m_write_buffer.end()) {
          // The request will depart at the next cycle
          req.depart = m_clk + 1;
          pending.push_back(req);
          return true;
        }
      }

      // Else, enqueue them to corresponding buffer based on request type id
      bool is_success = false;
      req.arrive = m_clk;
      if        (req.type_id == Request::Type::Read) {
        is_success = m_read_buffer.enqueue(req);
      } else if (req.type_id == Request::Type::Write) {
        is_success = m_write_buffer.enqueue(req);
      } else if (req.type_id == Request::Type::CuD){
        is_success = m_cud_buffer.enqueue(req);
      } else {
        throw std::runtime_error("Invalid request type!");
      }
      if (!is_success) {
        // We could not enqueue the request
        req.arrive = -1;
        return false;
      }

      return true;
    };

    bool priority_send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      bool is_success = false;
      is_success = m_priority_buffer.enqueue(req);
      return is_success;
    }

    void tick() override {
      m_clk++;
      CheckandPrintBuffers();

      // Update statistics
      s_queue_len += m_read_buffer.size() + m_write_buffer.size() + m_priority_buffer.size() + pending.size();
      s_read_queue_len += m_read_buffer.size() + pending.size();
      s_write_queue_len += m_write_buffer.size();
      s_priority_queue_len += m_priority_buffer.size();

      // 1. Serve completed reads
      serve_completed_reads();
      m_refresh->tick();

      // 2. Try to find a request to serve.
      ReqBuffer::iterator req_it;
      ReqBuffer* buffer = nullptr;
      schedule_cud();
      bool request_found = schedule_request(req_it, buffer);

      // 2.1 Take row policy action
      m_rowpolicy->update(request_found, req_it);

      // 3. Update all plugins
      for (auto plugin : m_plugins) {
        plugin->update(request_found, req_it);
      }

      // 4. Finally, issue the commands to serve the request
      if (request_found) {
        // If we find a real request to serve
        if (req_it->is_stat_updated == false) {
          update_request_stats(req_it);
        }
        //SYKIM
        // ProfileCommands(m_clk, req_it->addr, req_it->addr_vec, req_it->command, req_it->final_command);
        m_dram->issue_command(req_it->command, req_it->addr_vec);

        // If we are issuing the last command, set depart clock cycle and move the request to the pending queue
        if (req_it->command == req_it->final_command) {
          if (req_it->type_id == Request::Type::Read) {
            req_it->depart = m_clk + m_dram->m_read_latency;
            pending.push_back(*req_it);
          } else if (req_it->type_id == Request::Type::Write) {
            // TODO: Add code to update statistics
          }
          buffer->remove(req_it);
        } else {
          if (m_dram->m_command_meta(req_it->command).is_opening) {
            if (m_active_buffer.enqueue(*req_it)) {
              buffer->remove(req_it);
            }
          }
        }

      }
    };


  private:
    /**
     * @brief    Helper function to check if a request is hitting an open row
     * @details
     *
     */
    bool is_row_hit(ReqBuffer::iterator& req)
    {
        return m_dram->check_rowbuffer_hit(req->final_command, req->addr_vec);
    }
    /**
     * @brief    Helper function to check if a request is opening a row
     * @details
     *
    */
    bool is_row_open(ReqBuffer::iterator& req)
    {
        return m_dram->check_node_open(req->final_command, req->addr_vec);
    }

    /**
     * @brief
     * @details
     *
     */
    void update_request_stats(ReqBuffer::iterator& req)
    {
      req->is_stat_updated = true;

      if (req->type_id == Request::Type::Read)
      {
        if (is_row_hit(req)) {
          s_read_row_hits++;
          s_row_hits++;
          if (req->source_id != -1)
            s_read_row_hits_per_core[req->source_id]++;
        } else if (is_row_open(req)) {
          s_read_row_conflicts++;
          s_row_conflicts++;
          if (req->source_id != -1)
            s_read_row_conflicts_per_core[req->source_id]++;
        } else {
          s_read_row_misses++;
          s_row_misses++;
          if (req->source_id != -1)
            s_read_row_misses_per_core[req->source_id]++;
        }
      }
      else if (req->type_id == Request::Type::Write)
      {
        if (is_row_hit(req)) {
          s_write_row_hits++;
          s_row_hits++;
        } else if (is_row_open(req)) {
          s_write_row_conflicts++;
          s_row_conflicts++;
        } else {
          s_write_row_misses++;
          s_row_misses++;
        }
      }
    }

    /**
     * @brief    Helper function to serve the completed read requests
     * @details
     * This function is called at the beginning of the tick() function.
     * It checks the pending queue to see if the top request has received data from DRAM.
     * If so, it finishes this request by calling its callback and poping it from the pending queue.
     */
    void serve_completed_reads() {
      if (pending.size()) {
        // Check the first pending request
        auto& req = pending[0];
        if (req.depart <= m_clk) {
          // Request received data from dram
          if (req.depart - req.arrive > 1) {
            // Check if this requests accesses the DRAM or is being forwarded.
            // TODO add the stats back
            s_read_latency += req.depart - req.arrive;
          }

          if (req.callback) {
            // If the request comes from outside (e.g., processor), call its callback
            req.callback(req);
          }
          // Finally, remove this request from the pending queue
          pending.pop_front();
        }
      };
    };


    /**
     * @brief    Checks if we need to switch to write mode
     *
     */
    void set_write_mode() {
      if (!m_is_write_mode) {
        if ((m_write_buffer.size() > m_wr_high_watermark * m_write_buffer.max_size) || m_read_buffer.size() == 0) {
          m_is_write_mode = true;
        }
      } else {
        if ((m_write_buffer.size() < m_wr_low_watermark * m_write_buffer.max_size) && m_read_buffer.size() != 0) {
          m_is_write_mode = false;
        }
      }
    };


    /**
     * @brief    Helper function to find a request to schedule from the buffers.
     *
     */
    bool schedule_request(ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {
      bool request_found = false;
      // 2.1    First, check the act buffer to serve requests that are already activating (avoid useless ACTs)
      if (req_it= m_scheduler->get_best_request(m_active_buffer); req_it != m_active_buffer.end()) {
        if (m_dram->check_ready(req_it->command, req_it->addr_vec)) {
          request_found = true;
          req_buffer = &m_active_buffer;
        }
      }

      // 2.2    If no requests can be scheduled from the act buffer, check the rest of the buffers
      if (!request_found) {
        // 2.2.1    We first check the priority buffer to prioritize e.g., maintenance requests
        if (m_priority_buffer.size() != 0) {
          req_buffer = &m_priority_buffer;
          req_it = m_priority_buffer.begin();
          req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);

          request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
          if (!request_found & m_priority_buffer.size() != 0) {
            return false;
          }
        }

        // 2.2.1    If no request to be scheduled in the priority buffer, check the read and write buffers.
        if (!request_found) {
          // Query the write policy to decide which buffer to serve
          set_write_mode();
          auto& buffer = m_is_write_mode ? m_write_buffer : m_read_buffer;
          if (req_it = m_scheduler->get_best_request(buffer); req_it != buffer.end()) {
            request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
            req_buffer = &buffer;
          }
        }
      }

      // 2.3 If we find a request to schedule, we need to check if it will close an opened row in the active buffer.
      if (request_found) {
        if (m_dram->m_command_meta(req_it->command).is_closing) {
          auto& rowgroup = req_it->addr_vec;
          for (auto _it = m_active_buffer.begin(); _it != m_active_buffer.end(); _it++) {
            auto& _it_rowgroup = _it->addr_vec;
            bool is_matching = true;
            for (int i = 0; i < m_bank_addr_idx + 1 ; i++) {
              if (_it_rowgroup[i] != rowgroup[i] && _it_rowgroup[i] != -1 && rowgroup[i] != -1) {
                is_matching = false;
                break;
              }
            }
            if (is_matching) {
              request_found = false;
              break;
            }
          }
        }
      }

      return request_found;
    }

    void finalize() override {
      s_avg_read_latency = (float) s_read_latency / (float) s_num_read_reqs;

      s_queue_len_avg = (float) s_queue_len / (float) m_clk;
      s_read_queue_len_avg = (float) s_read_queue_len / (float) m_clk;
      s_write_queue_len_avg = (float) s_write_queue_len / (float) m_clk;
      s_priority_queue_len_avg = (float) s_priority_queue_len / (float) m_clk;
      double tCK_ps = m_dram->m_timing_vals("tCK_ps");

      // Print CuD profiling summary
      printf("\n");
      printf("+------------------------------------------------------------+\n");
      printf("|            CuD Profile                                     |\n");
      printf("+------------------------------------------------------------+\n");
      printf("|  CuD instructions completed        : %-26zu \n", s_cud_completed);
      printf("|    RowCopy(SRC)                      : %-26zu \n", s_cud_rowcopy_src_count);
      printf("|    RowCopy(DST)                      : %-26zu \n", s_cud_rowcopy_dst_count);
      printf("|    Majority3                         : %-26zu \n", s_cud_majority3_count);
      printf("|    Majority5                         : %-26zu \n", s_cud_majority5_count);
      printf("|  Total Rowcopy_ACT issued (CuD)    : %-26zu \n", s_cud_act_count_rowcopy);
      printf("|  Total Maj3_ACT issued (CuD)       : %-26zu \n", s_cud_act_count_maj3);
      printf("|  Total Maj5_ACT issued (CuD)       : %-26zu \n", s_cud_act_count_maj5);
      printf("|  Total PRE issued (CuD)            : %-26zu \n", s_cud_pre_count);
      printf("|  Total exec cycle (latency in ns)  : %-10zu (%f) \n", s_cud_total_exec_cycles, (double)s_cud_total_exec_cycles * tCK_ps / 1000.0);
      printf("|    [Per-opcode exec cycles/latency]                          \n");
      printf("|    RowCopy(SRC)                : %-26zu \n", s_cud_rowcopy_src_exec_cycles);
      printf("|    RowCopy(DST)                : %-26zu \n", s_cud_rowcopy_dst_exec_cycles);
      printf("|    Majority3                   : %-26zu \n", s_cud_majority3_exec_cycles);
      printf("|    Majority5                   : %-26zu \n", s_cud_majority5_exec_cycles);
      printf("+------------------------------------------------------------+\n");
      return;
    }

    /**
     *  Checks if CuD is executing
     *
     */
    bool is_cud_active() {
      return !(m_cud_inflight == nullptr);
    }

    void MangeCuDInst(Request& req) {
        const int CACT = m_cact_cmd_id;
        const int PRE  = m_dram->m_commands("PRE");
        const CuDInstDecoded d = CuDAddrDecoder::decode(req.cud_inst);
        const AddrVec_t& av   = req.addr_vec;

        switch (d.opcode) {
            case CuDOpcode::RowCopySrc:
                m_cud_cmd_queue.push({PRE,  av});
                m_cud_cmd_queue.push({CACT, av});
                s_cud_rowcopy_src_count++;
                s_cud_act_count_rowcopy++;
                break;
            case CuDOpcode::RowCopyDst:
                if (!d.is_last) {
                    m_cud_cmd_queue.push({CACT, av});
                    s_cud_act_count_rowcopy++;
                } else {
                    m_cud_cmd_queue.push({CACT, av});
                    m_cud_cmd_queue.push({PRE,  av});
                    s_cud_act_count_rowcopy++;
                }
                s_cud_rowcopy_dst_count++;
                break;
            case CuDOpcode::Majority3:
                m_cud_cmd_queue.push({PRE,  av});
                m_cud_cmd_queue.push({CACT, av});
                m_cud_cmd_queue.push({PRE,  av});
                s_cud_majority3_count++;
                s_cud_act_count_maj3++;
                break;
            case CuDOpcode::Majority5:
                m_cud_cmd_queue.push({PRE,  av});
                m_cud_cmd_queue.push({CACT, av});
                m_cud_cmd_queue.push({PRE,  av});
                s_cud_majority5_count++;
                s_cud_act_count_maj5++;
                break;
            default:
                printf("[CuD] MangeCuDInst: Invalid opcode! inst:%08x\n", req.cud_inst);
                break;
        }
            m_cud_inflight_opcode = d.opcode;
            m_cud_inst_start_clk  = m_clk;

            CUD_LOG("Manage CuD inst:%08x  opcode:%s  bg:%u  bank:%u  row:%u  num_cmds:%zu  clk:%ld\n",
                    req.cud_inst, get_CuD_opcode_name((int)d.opcode), d.bankgroup, d.bank, d.row,
                    m_cud_cmd_queue.size(), m_clk);
    }

    bool schedule_cud() {
      // Step 1. Expand next CuD request if queue is empty
      if (m_cud_cmd_queue.empty()) {
        if (m_cud_buffer.size() == 0) return false;
        Request& req   = *m_cud_buffer.begin();
        m_cud_inflight = &req;
        MangeCuDInst(req);
      }

      // Step 2. Try to issue front command
      CuDCmd& front = m_cud_cmd_queue.front();
      if (!m_dram->check_ready(front.cmd_id, front.addr_vec)) {
        s_cud_stall_cycles++;
        CUD_LOG("cmd:%s not ready, stall clk:%ld\n", get_command_name(front.cmd_id), m_clk);
        return false;
      }

      // Step 3. Issue command and update per-command stats
      m_dram->issue_command(front.cmd_id, front.addr_vec);
      if (front.cmd_id == m_dram->m_commands("PRE")) s_cud_pre_count++; // Need to count if PRE issued.

      CUD_LOG("Issue cmd:%s  clk:%ld\n", get_command_name(front.cmd_id), m_clk);
      m_cud_cmd_queue.pop();

      // Step 4. Check if instruction is complete
      if (m_cud_cmd_queue.empty() && m_cud_inflight != nullptr) {
        size_t exec_cycles = m_clk - m_cud_inst_start_clk;
        s_cud_total_exec_cycles += exec_cycles + 1;

        // Per-opcode execution cycle accumulation
        switch (m_cud_inflight_opcode) {
          case CuDOpcode::RowCopySrc: s_cud_rowcopy_src_exec_cycles += exec_cycles; break;
          case CuDOpcode::RowCopyDst: s_cud_rowcopy_dst_exec_cycles += exec_cycles; break;
          case CuDOpcode::Majority3:  s_cud_majority3_exec_cycles   += exec_cycles; break;
          case CuDOpcode::Majority5:  s_cud_majority5_exec_cycles   += exec_cycles; break;
          default: break;
        }
        s_cud_completed++;

        CUD_LOG("Completed inst:%08x  exec_cycles:%zu  clk:%ld\n",
                m_cud_inflight->cud_inst, exec_cycles, m_clk);

        if (m_cud_inflight->callback) m_cud_inflight->callback(*m_cud_inflight);
        m_cud_buffer.remove(m_cud_buffer.begin());
        m_cud_inflight        = nullptr;
        m_cud_inflight_opcode = CuDOpcode::Invalid;
      }

      return true;
    }

    void CheckandPrintBuffers() {
      size_t current_read = m_read_buffer.size();
      size_t current_write = m_write_buffer.size();
      size_t current_active = m_active_buffer.size();
      size_t current_priority = m_priority_buffer.size();
      size_t current_cud = m_cud_buffer.size();

      bool hasChanged = false;

      if (!m_is_buff_initialized) {
          hasChanged = true;
          m_is_buff_initialized = true;
      }
      else if (current_read != m_prev_read_size ||
               current_write != m_prev_write_size ||
               current_active != m_prev_active_size ||
               current_priority != m_prev_priority_size ||
               current_cud != m_prev_cud_size) {
          hasChanged = true;
      }
      if (hasChanged) {
          BUF_LOG("[CuD Profiler]:: Buffer size{CuD,RD,WR,ACT,PRI}:{%ld,%ld,%ld,%ld,%ld} at m_clk:%ld\n",
                 current_cud, current_read, current_write, current_active, current_priority, m_clk);

        m_prev_read_size = current_read;
        m_prev_write_size = current_write;
        m_prev_active_size = current_active;
        m_prev_priority_size = current_priority;
        m_prev_cud_size = current_cud;
    }

  }
  const char* get_command_name(int cmd_id) {
    try { return m_dram->m_commands(cmd_id).data(); }
    catch (...) { return "UNKNOWN"; }
  }
  const char* get_CuD_opcode_name (int cmd_code){
    switch (cmd_code)
    {
        case 1: return "RowCopy(SRC)";
        case 2: return "RowCopy(DST)";
        case 3: return "Majority3";
        case 4: return "Majority5";
        default: return "UNKNOWN";
    }
  }
  void ProfileCommands(long clk, Addr_t addr, AddrVec_t addr_vec, int cmd, int final_cmd){
    int m_ra_id, m_ba_id, m_bg_id, m_ro_id, m_co_id, m_ch_id;
    int rank_id, bank_id, bankgroup_id, row_id, col_id, channel_id;
        m_ra_id = m_dram->m_levels("rank");
        m_bg_id = m_dram->m_levels("bankgroup");
        m_ba_id = m_dram->m_levels("bank");
        m_ro_id = m_dram->m_levels("row");
        m_co_id = m_dram->m_levels("column");
        m_ch_id = m_dram->m_levels("channel");

        rank_id      = addr_vec[m_ra_id];
        bankgroup_id = addr_vec[m_bg_id];
        bank_id      = addr_vec[m_ba_id];
        row_id       = addr_vec[m_ro_id];
        col_id       = addr_vec[m_co_id];
        channel_id   = addr_vec[m_ch_id];

        printf("[CuD Profiler]:: Clk:%ld | Cmd:%s (Final:%s) | PhysAddr:%ld | "
               "Channel:%d, Rank:%d, BG:%d, Bank:%d, Row:%d, Col:%d\n",
               clk, get_command_name(cmd), get_command_name(final_cmd), addr,
               channel_id, rank_id, bankgroup_id, bank_id, row_id, col_id);
  }
};

}   // namespace Ramulator
