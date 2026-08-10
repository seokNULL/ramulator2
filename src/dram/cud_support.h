// src/dram/cud_support.h
//
// Shared helpers for adding CACT (CuD-ACT) support to any DRAM implementation.
//
// CuD (Compute-using-DRAM) activates rows for near-DRAM computation without a
// subsequent column access.  The key timing relaxation is that CACT→CACT and
// CACT→PRE use nRCD instead of nRC / nRAS, because there is no charge sharing
// from a column access between two consecutive CuD activations.
//
// ── What must be done per DRAM (compile-time, unavoidable) ──────────────────
//  1. Add "CACT" to m_commands / m_command_scopes ("row") / m_command_meta
//  2. Override: int get_cact_cmd_id() const override { return m_commands("CACT"); }
//
// ── What this header provides (runtime, shared across all DRAM types) ────────
//  CuDSupport::add_timing<T>()   — timing constraints via populate_timingcons()
//  CuDSupport::add_actions<T>()  — bank-level open / close state transitions
//  CuDSupport::add_preqs<T>()    — prerequisite command resolver
//
// ── Usage example in a DRAM's set_* methods ─────────────────────────────────
//   // set_timing_vals() — after the main populate_timingcons() call:
//   CuDSupport::add_timing<MyDRAM>(this, m_timing_vals, {
//       .nrcd       = "nRCD",          // CuD relaxed delay
//       .nrp        = "nRP",
//       .nrc        = "nRC",
//       .nrfc       = "nRFC",
//       .nrrd_s     = "nRRDS",
//       .nrrd_l     = "nRRDL",
//       .rank_level = "rank",          // "rank" or "pseudochannel"
//       .nfaw       = "nFAW",          // omit or leave "" to skip FAW constraint
//   });
//   // set_actions():
//   CuDSupport::add_actions<MyDRAM>(m_actions);
//   // set_preqs():
//   CuDSupport::add_preqs<MyDRAM>(m_preqs);
//
// ── Per-DRAM nrcd mapping ────────────────────────────────────────────────────
//   DDR4, DDR5 : nrcd = "nRCD"
//   HBM3       : nrcd = "nRCDRD"   (HBM3 splits nRCD into read/write)
//   GDDR6      : nrcd = "nRCDRD"
//   LPDDR5     : nrcd = "nRCD"
//
// Note: DRAM types with device-specific channel-level row-command latency
// (e.g., HBM3's 2-cycle ACT) must add those channel-level CACT entries
// directly in their own populate_timingcons() call — the helper only covers
// the universal rank/pseudochannel + bankgroup + bank levels.

#pragma once

#include <string_view>
#include <vector>

#include "dram/spec.h"
#include "dram/node.h"
#include "dram/lambdas.h"

namespace Ramulator {
namespace CuDSupport {

// Maps abstract CuD timing requirements to a specific DRAM's parameter names.
struct TimingConfig {
    // Core CuD relaxation — CACT→{ACT,CACT,PRE,RD,WR} uses this delay
    // instead of nRC (for ACT→ACT) or nRAS (for ACT→PRE).
    std::string_view nrcd;
    // Standard precharge penalty — PRE→CACT delay.
    std::string_view nrp;
    // Full row-cycle time — ACT→CACT at bank level (same as normal ACT→ACT).
    std::string_view nrc;
    // Refresh cycle time — REFab→CACT (same as REFab→ACT).
    std::string_view nrfc;
    // Short same-rank RAS-to-RAS rate — CACT↔ACT across banks.
    std::string_view nrrd_s;
    // Long within-bankgroup RAS-to-RAS rate — CACT↔ACT within a bankgroup.
    std::string_view nrrd_l;
    // Hierarchy level that acts as "rank" for inter-bank constraints.
    // Use "rank" for DDR4/DDR5, "pseudochannel" for HBM3.
    std::string_view rank_level;
    // Optional: 4-activation window timing parameter name.
    // Leave empty ("") to skip the FAW constraint for CACT.
    std::string_view nfaw = "";
};

// Add CACT timing constraints to dram->m_timing_cons.
//
// Call this at the END of set_timing_vals(), after the main populate_timingcons()
// call.  populate_timingcons() is additive (push_back) after the first resize,
// so calling it a second time here safely appends without overwriting.
template<typename DRAM_T>
void add_timing(DRAM_T* dram, SpecLUT<int>& tv, const TimingConfig& cfg) {
    int nrcd  = tv(cfg.nrcd);
    int nrp   = tv(cfg.nrp);
    int nrc   = tv(cfg.nrc);
    int nrfc  = tv(cfg.nrfc);
    int nrrds = tv(cfg.nrrd_s);
    int nrrdl = tv(cfg.nrrd_l);

    std::vector<TimingConsInitializer> entries = {
        // ── rank / pseudochannel level ────────────────────────────────────
        // ACT→CACT: same short-RRD rate as ACT→ACT
        {.level = cfg.rank_level, .preceding = {"ACT"},   .following = {"CACT"},        .latency = nrrds},
        // CACT→ACT/CACT: CuD operations at standard RRD rate
        {.level = cfg.rank_level, .preceding = {"CACT"},  .following = {"ACT", "CACT"}, .latency = nrrds},
        // CACT→PREA: relaxed — nRCD instead of nRAS
        {.level = cfg.rank_level, .preceding = {"CACT"},  .following = {"PREA"},         .latency = nrcd},
        // PREA→CACT: standard precharge penalty
        {.level = cfg.rank_level, .preceding = {"PREA"},  .following = {"CACT"},         .latency = nrp},
        // CACT→REFab: relaxed — nRCD instead of nRC
        {.level = cfg.rank_level, .preceding = {"CACT"},  .following = {"REFab"},        .latency = nrcd},
        // REFab→CACT: same as REFab→ACT
        {.level = cfg.rank_level, .preceding = {"REFab"}, .following = {"CACT"},         .latency = nrfc},

        // ── bankgroup level ───────────────────────────────────────────────
        // ACT→CACT: long-RRD rate (within bankgroup)
        {.level = "bankgroup",    .preceding = {"ACT"},   .following = {"CACT"},        .latency = nrrdl},
        // CACT→ACT/CACT: long-RRD rate
        {.level = "bankgroup",    .preceding = {"CACT"},  .following = {"ACT", "CACT"}, .latency = nrrdl},

        // ── bank level ────────────────────────────────────────────────────
        // ACT→CACT: full row-cycle time (same as normal same-bank ACT→ACT)
        {.level = "bank",         .preceding = {"ACT"},   .following = {"CACT"},                   .latency = nrc},
        // CACT→ACT/CACT: KEY RELAXATION — nRCD instead of nRC
        {.level = "bank",         .preceding = {"CACT"},  .following = {"ACT", "CACT"},             .latency = nrcd},
        // CACT→PRE: KEY RELAXATION — nRCD instead of nRAS
        {.level = "bank",         .preceding = {"CACT"},  .following = {"PRE"},                     .latency = nrcd},
        // CACT→column: nRCD (standard row-to-column latency)
        {.level = "bank",         .preceding = {"CACT"},  .following = {"RD","RDA","WR","WRA"},     .latency = nrcd},
        // PRE→CACT: standard precharge penalty
        {.level = "bank",         .preceding = {"PRE"},   .following = {"CACT"},                   .latency = nrp},
    };

    // Optional 4-activation window for CACT (mirrors the normal ACT FAW rule)
    if (!cfg.nfaw.empty()) {
        int nfaw = tv(cfg.nfaw);
        entries.push_back({
            .level     = cfg.rank_level,
            .preceding = {"CACT"},
            .following = {"CACT"},
            .latency   = nfaw,
            .window    = 4,
        });
    }

    populate_timingcons(dram, std::move(entries));
}

// Assign CACT the same bank open/close state transitions as ACT.
template<typename DRAM_T>
void add_actions(FuncMatrix<ActionFunc_t<typename DRAM_T::Node>>& m_actions) {
    m_actions[DRAM_T::m_levels["bank"]][DRAM_T::m_commands["CACT"]] =
        Lambdas::Action::Bank::ACT<DRAM_T>;
}

// Require the bank row to be open before issuing CACT (same as normal ACT).
template<typename DRAM_T>
void add_preqs(FuncMatrix<PreqFunc_t<typename DRAM_T::Node>>& m_preqs) {
    m_preqs[DRAM_T::m_levels["bank"]][DRAM_T::m_commands["CACT"]] =
        Lambdas::Preq::Bank::RequireRowOpen<DRAM_T>;
}

} // namespace CuDSupport
} // namespace Ramulator
