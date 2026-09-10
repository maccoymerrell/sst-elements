// Copyright 2009-2025 NTESS. Under the terms
// of Contract DE-NA0003525 with NTESS, the U.S.
// Government retains certain rights in this software.
//
// Copyright (c) 2009-2025, NTESS
// All rights reserved.
//
// Portions are copyright of other developers:
// See the file CONTRIBUTORS.TXT in the top level directory
// of the distribution for more information.
//
// This file is part of the SST software package. For license
// information, see the LICENSE file in the top level directory of the
// distribution.

#ifndef _H_VANADIS_FDIP
#define _H_VANADIS_FDIP

// FETCH-DIRECTED INSTRUCTION PREFETCHING
//
//   Reinman, Calder and Austin, "Fetch Directed Instruction Prefetching",
//   MICRO-32, 1999.
//
// The idea in one sentence: the branch predictor does not have to wait for the
// instruction cache. Give it its own address stream, let it run ahead, put the
// fetch addresses it produces in a queue -- the FETCH TARGET QUEUE -- and use
// that queue as a list of the cache lines fetch is about to want. Prefetch
// them. When fetch arrives the line is there.
//
// The paper's front end is a fetch target buffer (a BTB that answers "where is
// the next taken branch, and where does it go" for a whole fetch block at a
// time), a direction predictor, and a return address stack. Those three produce
// one FETCH BLOCK per cycle into the FTQ; an enqueued prefetch engine walks the
// FTQ and issues cache-line fetches for the blocks fetch has not reached yet.
// On a misprediction the FTQ is thrown away and the run-ahead restarts at the
// corrected address. That is what this file implements.
//
// WHAT IS DIFFERENT HERE, and it has to be said plainly. Vanadis has no fetch
// stage: the decoder fetches, decodes and predicts in one tick, and its
// direction predictor (TAGE-SC-L, vbranch/) is consulted AFTER the instruction
// has been decoded -- it is handed the branch's class and its static target by
// the decoded micro-op. A predictor that is asked only about instructions that
// have already been decoded cannot run ahead of the instruction cache, because
// everything it can be asked about is already in the micro-op cache and needs
// no fetch at all.
//
// So this engine carries its own next-block predictor -- an FTB trained at
// decode, a gshare direction table and a return address stack trained at
// retire -- which is exactly the structure the paper's front end has, and it
// leaves the core's TAGE-SC-L alone. The consequence is stated rather than
// hidden: the two predictors can disagree, and when they do the FTQ is on a
// path the core will not take, so its prefetches are useless ones. They are
// never wrong ANSWERS -- an FDIP prefetch cannot change what the machine
// computes, because the bytes it brings back are dropped. The only thing it
// changes is where the line is when the demand fetch asks for it.
//
// WHERE A PREFETCH GOES. The core's own line buffer (the loader's predecode
// cache, four lines) is left alone: a prefetch's response is DISCARDED, so the
// prefetch's whole effect is that the line is now in the L1I -- which is what
// the paper prefetches into. The demand fetch that follows still goes through
// the instruction MMU and still reads the L1I; it just hits.

#include "vbranch/vbranchunit.h"
#include "vinsloader.h"

#include <cstdint>
#include <deque>
#include <list>
#include <map>
#include <unordered_map>
#include <vector>

namespace SST {
namespace Vanadis {

// One fetch block: a run of instruction bytes, ending either at a predicted
// taken branch or at the end of the cache line, so a block never spans two
// lines and an FTQ entry names exactly one line to prefetch.
struct VanadisFTQEntry {
    uint64_t start    = 0;      // first byte
    uint64_t end      = 0;      // one past the last byte
    uint64_t line     = 0;      // the line this block lives in
    bool     pf_done  = false;  // the prefetch engine has dealt with this entry
};

// The counters the engine keeps. Registered by the decoder, which owns the
// statistics; the engine only adds to them.
struct VanadisFDIPStats {
    Statistic<uint64_t>* ftq_occupancy       = nullptr;
    Statistic<uint64_t>* ftq_full_cycles     = nullptr;
    Statistic<uint64_t>* ftq_empty_cycles    = nullptr;
    Statistic<uint64_t>* blocks_produced     = nullptr;
    Statistic<uint64_t>* runahead_blocked    = nullptr;
    Statistic<uint64_t>* ftq_flushes         = nullptr;
    Statistic<uint64_t>* ftq_flushed_entries = nullptr;
    Statistic<uint64_t>* ftb_hits            = nullptr;
    Statistic<uint64_t>* ftb_misses          = nullptr;
    Statistic<uint64_t>* ftb_evictions       = nullptr;
    Statistic<uint64_t>* ras_pushes          = nullptr;
    Statistic<uint64_t>* ras_pops            = nullptr;
    Statistic<uint64_t>* ras_empty           = nullptr;
    Statistic<uint64_t>* pf_issued           = nullptr;
    Statistic<uint64_t>* pf_useful           = nullptr;
    Statistic<uint64_t>* pf_late             = nullptr;
    Statistic<uint64_t>* pf_useless          = nullptr;
    Statistic<uint64_t>* pf_filtered         = nullptr;
    Statistic<uint64_t>* pf_in_line_buffer   = nullptr;
    Statistic<uint64_t>* pf_demand_pending   = nullptr;
    Statistic<uint64_t>* pf_refused          = nullptr;
    Statistic<uint64_t>* pf_dropped_response = nullptr;
};

class VanadisFDIP : public VanadisPrefetchSink
{
public:
    struct Config {
        bool     enable            = true;
        uint32_t ftq_entries       = 32;
        uint32_t blocks_per_cycle  = 2;
        uint32_t prefetch_per_cycle = 2;
        uint32_t max_outstanding   = 8;
        uint32_t ftb_entries       = 4096;
        uint32_t ras_entries       = 32;
        uint32_t gshare_entries    = 16384;
        uint32_t history_bits      = 14;
        uint32_t filter_sets       = 64;
        uint32_t filter_ways       = 8;
        uint64_t line_width        = 64;
    };

    VanadisFDIP(const Config& cfg, VanadisInstructionLoader* loader, SST::Output* out) :
        cfg_(cfg),
        loader_(loader),
        output_(out)
    {
        line_mask_    = ~(cfg_.line_width - 1);
        ghist_mask_   = (cfg_.history_bits >= 64) ? ~0ull : ((1ull << cfg_.history_bits) - 1);
        gshare_mask_  = cfg_.gshare_entries - 1;
        gshare_.assign(cfg_.gshare_entries, 1);     // weakly not-taken, as a cold table is
        ras_spec_.assign(cfg_.ras_entries, 0);
        ras_arch_.assign(cfg_.ras_entries, 0);
        filter_.assign((size_t)cfg_.filter_sets * cfg_.filter_ways, FilterEntry());
        if ( cfg_.enable ) { loader_->setPrefetchSink(this); }
    }

    ~VanadisFDIP() override {}

    void setStatistics(const VanadisFDIPStats& s) { stats_ = s; }

    bool enabled() const { return cfg_.enable; }

    // ---------------------------------------------------------------- decode

    // Called every time the decoder walks a branch micro-op out of the
    // micro-op cache. This is where the FTB learns that there IS a branch at
    // this address, how wide it is and, for a direct branch, where it goes.
    void observeBranchAtDecode(
        const uint64_t pc, const VanadisBranchClass cls, const uint64_t width, const bool has_static,
        const uint64_t static_target)
    {
        if ( !cfg_.enable ) { return; }

        FTBEntry e;
        e.cls        = cls;
        e.width      = (uint8_t)width;
        e.target     = has_static ? static_target : 0;
        e.has_target = has_static;

        auto found = ftb_.find(pc);
        if ( found != ftb_.end() ) {
            // Keep a target learned at retire for an indirect branch; decode
            // has nothing better to say about one.
            if ( !e.has_target && found->second.has_target ) {
                e.target     = found->second.target;
                e.has_target = true;
            }
            found->second = e;
            touchFTB(pc);
        }
        else {
            insertFTB(pc, e);
        }
    }

    // ---------------------------------------------------------------- retire

    // Program order, with the outcome known. Trains the direction table and
    // the architected history and return stack, and gives an indirect branch's
    // FTB entry the target it actually took.
    void branchRetired(const uint64_t pc, const VanadisBranchClass cls, const bool taken, const uint64_t target)
    {
        if ( !cfg_.enable ) { return; }

        if ( VanadisBranchClass::CONDITIONAL == cls ) {
            const uint64_t idx = gshareIndex(pc, ghist_arch_);
            uint8_t&       c   = gshare_[idx];
            if ( taken ) { if ( c < 3 ) { ++c; } }
            else         { if ( c > 0 ) { --c; } }
        }

        ghist_arch_ = ((ghist_arch_ << 1) | (taken ? 1ull : 0ull)) & ghist_mask_;

        auto found = ftb_.find(pc);
        if ( found != ftb_.end() ) {
            if ( taken ) {
                found->second.target     = target;
                found->second.has_target = true;
            }
            touchFTB(pc);
        }

        // The architected return stack: the copy a flush restores from.
        if ( isCall(cls) ) {
            auto fe = ftb_.find(pc);
            const uint64_t ret = (fe != ftb_.end()) ? (pc + fe->second.width) : (pc + 4);
            ras_arch_[ras_arch_top_] = ret;
            ras_arch_top_            = (ras_arch_top_ + 1) % cfg_.ras_entries;
            if ( ras_arch_depth_ < cfg_.ras_entries ) { ++ras_arch_depth_; }
        }
        else if ( VanadisBranchClass::RETURN == cls ) {
            if ( ras_arch_depth_ > 0 ) {
                ras_arch_top_ = (ras_arch_top_ + cfg_.ras_entries - 1) % cfg_.ras_entries;
                --ras_arch_depth_;
            }
        }
    }

    // ----------------------------------------------------------------- flush

    // Every redirect the core makes -- a branch misprediction, a system call
    // return, a fault, a thread start -- lands here. The FTQ described a path
    // the machine is not taking, so it goes, and the run-ahead's speculative
    // history and return stack go back to the architected copies, which is
    // what the core's own predictor does at the same moment.
    void flush(const uint64_t new_ip)
    {
        if ( !cfg_.enable ) { return; }

        if ( !ftq_.empty() ) {
            if ( stats_.ftq_flushed_entries ) { stats_.ftq_flushed_entries->addData(ftq_.size()); }
            ftq_.clear();
        }
        if ( stats_.ftq_flushes ) { stats_.ftq_flushes->addData(1); }

        ghist_        = ghist_arch_;
        ras_spec_     = ras_arch_;
        ras_spec_top_ = ras_arch_top_;
        ras_depth_    = ras_arch_depth_;
        ftq_pc_       = new_ip;
        blocked_      = false;
    }

    // ------------------------------------------------------------ every cycle

    void tick(const uint64_t cycle, const uint64_t fetch_ip)
    {
        if ( !cfg_.enable ) { return; }

        cycle_ = cycle;

        retire(fetch_ip);
        runAhead();
        prefetch();

        if ( stats_.ftq_occupancy ) { stats_.ftq_occupancy->addData(ftq_.size()); }
        if ( ftq_.empty() && stats_.ftq_empty_cycles ) { stats_.ftq_empty_cycles->addData(1); }
        if ( (ftq_.size() >= cfg_.ftq_entries) && stats_.ftq_full_cycles ) { stats_.ftq_full_cycles->addData(1); }
    }

    // ------------------------------------------------ the loader talks to us

    // A prefetched line came back. The bytes are dropped by the loader; all
    // that is recorded is that the line is now one level closer.
    void prefetchArrived(const uint64_t line) override
    {
        if ( !cfg_.enable ) { return; }
        if ( stats_.pf_dropped_response ) { stats_.pf_dropped_response->addData(1); }
        FilterEntry* e = filterFind(line);
        if ( (nullptr != e) && (PF_INFLIGHT == e->state) ) { e->state = PF_ARRIVED; }
    }

    // A demand fetch is about to be issued for this line. This is where a
    // prefetch is judged: the line was prefetched and had arrived (useful), was
    // prefetched and is still in the air (late -- the miss is shortened, not
    // removed), or was not prefetched at all.
    void demandFetch(const uint64_t line) override
    {
        if ( !cfg_.enable ) { return; }
        FilterEntry* e = filterFind(line);
        if ( nullptr != e ) {
            if ( PF_ARRIVED == e->state ) {
                if ( stats_.pf_useful ) { stats_.pf_useful->addData(1); }
            }
            else if ( PF_INFLIGHT == e->state ) {
                if ( stats_.pf_useful ) { stats_.pf_useful->addData(1); }
                if ( stats_.pf_late ) { stats_.pf_late->addData(1); }
            }
            e->state = DEMANDED;
            touchFilter(e);
        }
        else {
            filterInsert(line, DEMANDED);
        }
    }

private:
    struct FTBEntry {
        uint64_t           target     = 0;
        VanadisBranchClass cls        = VanadisBranchClass::CONDITIONAL;
        uint8_t            width      = 4;
        bool               has_target = false;
    };

    enum FilterState : uint8_t { EMPTY = 0, PF_INFLIGHT = 1, PF_ARRIVED = 2, DEMANDED = 3 };

    struct FilterEntry {
        uint64_t line  = 0;
        uint64_t lru   = 0;
        uint8_t  state = EMPTY;
    };

    static bool isCall(const VanadisBranchClass c)
    {
        return (VanadisBranchClass::DIRECT_CALL == c) || (VanadisBranchClass::INDIRECT_CALL == c);
    }

    static bool isIndirect(const VanadisBranchClass c)
    {
        return (VanadisBranchClass::INDIRECT_JUMP == c) || (VanadisBranchClass::INDIRECT_CALL == c);
    }

    uint64_t gshareIndex(const uint64_t pc, const uint64_t hist) const
    {
        return ((pc >> 1) ^ hist) & gshare_mask_;
    }

    // ---- the FTQ head follows the decoder ---------------------------------

    // The decoder's instruction pointer is the fetch stream. Entries the fetch
    // has walked past are retired from the head; if the fetch is at an address
    // no entry covers then the run-ahead is on a different path from the core
    // and the queue is thrown away.
    void retire(const uint64_t fetch_ip)
    {
        // An empty queue carries no path, so there is nothing to be blocked
        // on and nothing to compare against: the run-ahead simply restarts
        // where the fetch is.
        if ( ftq_.empty() ) {
            ftq_pc_  = fetch_ip;
            blocked_ = false;
            return;
        }

        size_t hit = ftq_.size();
        for ( size_t i = 0; i < ftq_.size(); ++i ) {
            if ( (fetch_ip >= ftq_[i].start) && (fetch_ip < ftq_[i].end) ) { hit = i; break; }
        }

        if ( hit == ftq_.size() ) {
            // Not on this path. Either the core was redirected without telling
            // us, or the run-ahead followed a stale FTB entry.
            flush(fetch_ip);
            return;
        }

        for ( size_t i = 0; i < hit; ++i ) { ftq_.pop_front(); }
    }

    // ---- the predictor runs ahead -----------------------------------------

    void runAhead()
    {
        if ( blocked_ ) {
            if ( stats_.runahead_blocked ) { stats_.runahead_blocked->addData(1); }
            return;
        }

        for ( uint32_t n = 0; n < cfg_.blocks_per_cycle; ++n ) {
            if ( ftq_.size() >= cfg_.ftq_entries ) { return; }

            const uint64_t pc       = ftq_pc_;
            const uint64_t line     = pc & line_mask_;
            const uint64_t line_end = line + cfg_.line_width;

            // The first branch the FTB knows about at or after pc, inside this
            // line. A line with no such branch falls through to the next one,
            // which is the assumption a BTB-directed front end makes and the
            // reason a straight-line region prefetches perfectly.
            auto     itr    = ftb_.lower_bound(pc);
            bool     is_br  = false;
            uint64_t br_pc  = 0;
            FTBEntry br;

            if ( (itr != ftb_.end()) && (itr->first < line_end) ) {
                is_br = true;
                br_pc = itr->first;
                br    = itr->second;
                touchFTB(br_pc);
            }

            if ( !is_br ) {
                if ( stats_.ftb_misses ) { stats_.ftb_misses->addData(1); }
                push(pc, line_end, line);
                ftq_pc_ = line_end;
                continue;
            }

            if ( stats_.ftb_hits ) { stats_.ftb_hits->addData(1); }

            const uint64_t br_end = br_pc + br.width;
            bool           taken  = true;

            if ( VanadisBranchClass::CONDITIONAL == br.cls ) {
                taken = (gshare_[gshareIndex(br_pc, ghist_)] >= 2);
            }

            if ( !taken ) {
                ghist_ = ((ghist_ << 1) | 0ull) & ghist_mask_;
                push(pc, br_end, line);
                ftq_pc_ = br_end;
                continue;
            }

            uint64_t target = br.target;
            if ( VanadisBranchClass::RETURN == br.cls ) {
                if ( ras_depth_ > 0 ) {
                    ras_spec_top_ = (ras_spec_top_ + cfg_.ras_entries - 1) % cfg_.ras_entries;
                    --ras_depth_;
                    target = ras_spec_[ras_spec_top_];
                    if ( stats_.ras_pops ) { stats_.ras_pops->addData(1); }
                }
                else {
                    if ( stats_.ras_empty ) { stats_.ras_empty->addData(1); }
                    target = br.has_target ? br.target : 0;
                }
            }
            else if ( isIndirect(br.cls) && !br.has_target ) {
                target = 0;
            }

            if ( isCall(br.cls) ) {
                ras_spec_[ras_spec_top_] = br_end;
                ras_spec_top_            = (ras_spec_top_ + 1) % cfg_.ras_entries;
                if ( ras_depth_ < cfg_.ras_entries ) { ++ras_depth_; }
                if ( stats_.ras_pushes ) { stats_.ras_pushes->addData(1); }
            }

            ghist_ = ((ghist_ << 1) | 1ull) & ghist_mask_;
            push(pc, br_end, line);

            if ( 0 == target ) {
                // An indirect branch with no target yet. There is no honest
                // guess to make, so the run-ahead stops until the core
                // redirects it -- an FTQ that ran on would be prefetching
                // whatever address zero happened to name.
                blocked_ = true;
                return;
            }

            ftq_pc_ = target;
        }
    }

    void push(const uint64_t start, const uint64_t end, const uint64_t line)
    {
        VanadisFTQEntry e;
        e.start = start;
        e.end   = end;
        e.line  = line;
        ftq_.push_back(e);
        if ( stats_.blocks_produced ) { stats_.blocks_produced->addData(1); }
    }

    // ---- the prefetch engine walks the FTQ --------------------------------

    void prefetch()
    {
        uint32_t issued = 0;

        for ( auto& e : ftq_ ) {
            if ( issued >= cfg_.prefetch_per_cycle ) { return; }
            if ( e.pf_done ) { continue; }

            // Already in the core's line buffer: nothing to fetch.
            if ( loader_->linePresent(e.line) ) {
                e.pf_done = true;
                if ( stats_.pf_in_line_buffer ) { stats_.pf_in_line_buffer->addData(1); }
                continue;
            }

            // A demand fetch is already in the air for it.
            if ( loader_->demandPendingLine(e.line) ) {
                e.pf_done = true;
                if ( stats_.pf_demand_pending ) { stats_.pf_demand_pending->addData(1); }
                continue;
            }

            // The engine's own record of what it has recently sent to, or seen
            // come from, the L1I. This is the stand-in for the cache-tag probe
            // the paper's prefetch engine does: the core cannot read the L1I's
            // tags from here, and re-sending a line the L1I certainly holds
            // would be pure traffic.
            FilterEntry* f = filterFind(e.line);
            if ( nullptr != f ) {
                e.pf_done = true;
                touchFilter(f);
                if ( stats_.pf_filtered ) { stats_.pf_filtered->addData(1); }
                continue;
            }

            if ( loader_->outstandingPrefetches() >= cfg_.max_outstanding ) {
                if ( stats_.pf_refused ) { stats_.pf_refused->addData(1); }
                return;
            }

            loader_->requestPrefetchLine(e.line);
            filterInsert(e.line, PF_INFLIGHT);
            e.pf_done = true;
            ++issued;
            if ( stats_.pf_issued ) { stats_.pf_issued->addData(1); }
        }
    }

    // ---- the FTB's least-recently-used replacement ------------------------

    void insertFTB(const uint64_t pc, const FTBEntry& e)
    {
        if ( ftb_.size() >= cfg_.ftb_entries ) {
            const uint64_t victim = ftb_lru_.back();
            ftb_lru_.pop_back();
            ftb_lru_pos_.erase(victim);
            ftb_.erase(victim);
            if ( stats_.ftb_evictions ) { stats_.ftb_evictions->addData(1); }
        }
        ftb_[pc] = e;
        ftb_lru_.push_front(pc);
        ftb_lru_pos_[pc] = ftb_lru_.begin();
    }

    void touchFTB(const uint64_t pc)
    {
        auto p = ftb_lru_pos_.find(pc);
        if ( p == ftb_lru_pos_.end() ) { return; }
        ftb_lru_.splice(ftb_lru_.begin(), ftb_lru_, p->second);
        p->second = ftb_lru_.begin();
    }

    // ---- the prefetch filter ----------------------------------------------

    FilterEntry* filterFind(const uint64_t line)
    {
        const size_t set  = (size_t)((line / cfg_.line_width) % cfg_.filter_sets);
        const size_t base = set * cfg_.filter_ways;
        for ( size_t w = 0; w < cfg_.filter_ways; ++w ) {
            FilterEntry& f = filter_[base + w];
            if ( (EMPTY != f.state) && (f.line == line) ) { return &f; }
        }
        return nullptr;
    }

    void touchFilter(FilterEntry* f) { f->lru = ++filter_clock_; }

    void filterInsert(const uint64_t line, const uint8_t state)
    {
        const size_t set  = (size_t)((line / cfg_.line_width) % cfg_.filter_sets);
        const size_t base = set * cfg_.filter_ways;

        size_t victim = base;
        for ( size_t w = 0; w < cfg_.filter_ways; ++w ) {
            FilterEntry& f = filter_[base + w];
            if ( EMPTY == f.state ) { victim = base + w; break; }
            if ( f.lru < filter_[victim].lru ) { victim = base + w; }
        }

        FilterEntry& f = filter_[victim];
        // A prefetched line thrown out of the record without a demand ever
        // asking for it: the prefetch bought nothing.
        if ( ((PF_INFLIGHT == f.state) || (PF_ARRIVED == f.state)) && stats_.pf_useless ) {
            stats_.pf_useless->addData(1);
        }
        f.line  = line;
        f.state = state;
        f.lru   = ++filter_clock_;
    }

    Config                    cfg_;
    VanadisInstructionLoader* loader_ = nullptr;
    SST::Output*              output_ = nullptr;
    VanadisFDIPStats          stats_;

    uint64_t line_mask_    = ~63ull;
    uint64_t ghist_mask_   = 0;
    uint64_t gshare_mask_  = 0;
    uint64_t cycle_        = 0;

    std::deque<VanadisFTQEntry> ftq_;
    uint64_t                    ftq_pc_  = 0;
    bool                        blocked_ = false;

    std::map<uint64_t, FTBEntry>                            ftb_;
    std::list<uint64_t>                                     ftb_lru_;
    std::unordered_map<uint64_t, std::list<uint64_t>::iterator> ftb_lru_pos_;

    std::vector<uint8_t> gshare_;
    uint64_t             ghist_      = 0;
    uint64_t             ghist_arch_ = 0;

    std::vector<uint64_t> ras_spec_;
    std::vector<uint64_t> ras_arch_;
    uint32_t              ras_spec_top_   = 0;
    uint32_t              ras_depth_      = 0;
    uint32_t              ras_arch_top_   = 0;
    uint32_t              ras_arch_depth_ = 0;

    std::vector<FilterEntry> filter_;
    uint64_t                 filter_clock_ = 0;
};

} // namespace Vanadis
} // namespace SST

#endif
