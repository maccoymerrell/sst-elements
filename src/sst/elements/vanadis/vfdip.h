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

// THE FETCH STAGE, AND THE PREFETCHING IT PAYS FOR.
//
// A modern processor predicts in two stages. At FETCH, one branch target
// buffer lookup per cycle says where the block being fetched ends, whether a
// branch ends it, and where that branch goes; the buffer's entry also carries
// the base direction predictor's two bits per branch, so one lookup gives both
// the target and the direction. Those answers steer fetch and are written into
// the FETCH TARGET QUEUE. At DECODE the main predictor -- TAGE's tagged
// tables, the statistical corrector, the loop predictor -- answers again and
// overrides the buffer when it disagrees, at the cost of the bubble between
// the two stages. The branch's resolution repairs both.
//
// This file is the first of those stages and the queue between them. It holds:
//
//   * the run-ahead pointer and the FETCH TARGET QUEUE, a queue of fetch
//     blocks. A block never spans a cache line, so one entry names exactly one
//     line, which is what makes the prefetch engine's walk trivial;
//   * the queue of the fetch stage's branch predictions, in fetch order, which
//     the decode stage consumes one at a time;
//   * the prefetch engine and its filter.
//
// The branch target buffer, the base bimodal counters that live in its
// entries, the return address stack and the tagged predictor are all in the
// branch unit (vbranch/), reached through VanadisBranchUnit. There is no
// second direction predictor here and no second target buffer: the fetch stage
// and the core predict from the same structures, which is what makes the
// queue's contents the path the core will actually take.
//
// FETCH-DIRECTED INSTRUCTION PREFETCHING is what the queue is then used for:
//
//   Reinman, Calder and Austin, "Fetch Directed Instruction Prefetching",
//   MICRO-32, 1999, and "Optimizations Enabled by a Decoupled Front-End
//   Architecture", UCSD CS2000-0645.
//
// The branch predictor does not have to wait for the instruction cache. Give
// it its own address stream, let it run ahead, and use the queue it fills as a
// list of the cache lines fetch is about to want. Prefetch them. When fetch
// arrives the line is there.
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

// What the fetch stage predicted about one branch, waiting for the decode
// stage to reach it.
struct VanadisFetchStageBranch {
    VanadisBranchCheckpoint ckpt;
    uint64_t                pc    = 0;
    uint64_t                next  = 0;
    bool                    taken = false;
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
    Statistic<uint64_t>* btb_bubbles         = nullptr;
    Statistic<uint64_t>* fetch_branches      = nullptr;
    Statistic<uint64_t>* decode_hit          = nullptr;
    Statistic<uint64_t>* decode_miss         = nullptr;
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
        uint32_t l2_bubble         = 1;
        uint32_t filter_sets       = 64;
        uint32_t filter_ways       = 8;
        uint64_t line_width        = 64;
    };

    VanadisFDIP(const Config& cfg, VanadisInstructionLoader* loader, VanadisBranchUnit* bp, SST::Output* out) :
        cfg_(cfg),
        loader_(loader),
        bp_(bp),
        output_(out)
    {
        line_mask_ = ~(cfg_.line_width - 1);
        filter_.assign((size_t)cfg_.filter_sets * cfg_.filter_ways, FilterEntry());
        if ( cfg_.enable ) { loader_->setPrefetchSink(this); }
    }

    ~VanadisFDIP() override {}

    void setStatistics(const VanadisFDIPStats& s) { stats_ = s; }

    bool enabled() const { return cfg_.enable; }

    // ---------------------------------------------------------------- decode

    // The decode stage has reached a branch at `pc`. If the fetch stage
    // predicted it, its record comes back here and the decode stage overrides
    // it or accepts it. If it did not -- because the branch target buffer had
    // not marked this address -- the decode stage has to make the prediction
    // itself, and the run-ahead, which walked straight past a branch, is on a
    // path that never existed.
    bool takeBranchPrediction(const uint64_t pc, VanadisFetchStageBranch* out)
    {
        if ( !cfg_.enable ) { return false; }

        if ( !pending_.empty() && (pending_.front().pc == pc) ) {
            *out = pending_.front();
            pending_.pop_front();
            if ( stats_.decode_hit ) { stats_.decode_hit->addData(1); }
            return true;
        }

        // The fetch stage has nothing for this address. Either the branch is
        // not marked in the buffer -- in which case the run-ahead was right to
        // walk past it and nothing here is stale -- or the run-ahead is
        // elsewhere. The decode stage knows which, and calls resteerBefore()
        // for the second case.
        if ( stats_.decode_miss ) { stats_.decode_miss->addData(1); }
        return false;
    }

    // The decode stage is about to make a fetch-stage prediction of its own.
    // Records are allocated in fetch order, so everything the fetch stage
    // predicted and the decode stage never reached has to go first.
    void resteerBefore()
    {
        if ( !cfg_.enable ) { return; }
        pending_.clear();
        ftq_.clear();
        if ( nullptr != bp_ ) { bp_->discardRunAhead(); }
        blocked_ = false;
        bubble_  = 0;
    }

    // ----------------------------------------------------------------- flush

    // Every redirect the core makes -- a branch misprediction, a decode-stage
    // override, a branch the buffer did not know about, a fault, a system call
    // resume, a thread start -- lands here. The queue described a path the
    // machine is not taking, so it goes, and every prediction the fetch stage
    // made that the decode stage never consumed goes with it.
    void flush(const uint64_t new_ip)
    {
        if ( !cfg_.enable ) { return; }

        if ( !ftq_.empty() ) {
            if ( stats_.ftq_flushed_entries ) { stats_.ftq_flushed_entries->addData(ftq_.size()); }
            ftq_.clear();
        }
        if ( stats_.ftq_flushes ) { stats_.ftq_flushes->addData(1); }

        pending_.clear();
        if ( nullptr != bp_ ) { bp_->discardRunAhead(); }

        ftq_pc_  = new_ip;
        blocked_ = false;
        bubble_  = 0;
    }

    // ------------------------------------------------------------ every cycle

    void tick(const uint64_t cycle, const uint64_t fetch_ip)
    {
        if ( !cfg_.enable ) { return; }

        cycle_ = cycle;

        follow(fetch_ip);
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
    enum FilterState : uint8_t { EMPTY = 0, PF_INFLIGHT = 1, PF_ARRIVED = 2, DEMANDED = 3 };

    struct FilterEntry {
        uint64_t line  = 0;
        uint64_t lru   = 0;
        uint8_t  state = EMPTY;
    };

    // ---- the FTQ head follows the decode stage ----------------------------

    void follow(const uint64_t fetch_ip)
    {
        // An empty queue carries no path, so there is nothing to be compared
        // against: the run-ahead simply restarts where the decode stage is.
        if ( ftq_.empty() ) {
            if ( ftq_pc_ != fetch_ip ) {
                pending_.clear();
                if ( nullptr != bp_ ) { bp_->discardRunAhead(); }
            }
            ftq_pc_  = fetch_ip;
            blocked_ = false;
            return;
        }

        size_t hit = ftq_.size();
        for ( size_t i = 0; i < ftq_.size(); ++i ) {
            if ( (fetch_ip >= ftq_[i].start) && (fetch_ip < ftq_[i].end) ) { hit = i; break; }
        }

        if ( hit == ftq_.size() ) {
            // Not on this path: the core was redirected without this stage
            // being told, or the run-ahead followed an entry that has since
            // been retrained.
            flush(fetch_ip);
            return;
        }

        for ( size_t i = 0; i < hit; ++i ) { ftq_.pop_front(); }
    }

    // ---- the fetch stage runs ahead ---------------------------------------

    void runAhead()
    {
        if ( blocked_ ) {
            if ( stats_.runahead_blocked ) { stats_.runahead_blocked->addData(1); }
            return;
        }

        if ( nullptr == bp_ ) { return; }

        for ( uint32_t n = 0; n < cfg_.blocks_per_cycle; ++n ) {
            if ( bubble_ > 0 ) {
                // A prediction bubble charged by a second-level buffer hit:
                // this cycle's block does not appear.
                --bubble_;
                return;
            }

            if ( ftq_.size() >= cfg_.ftq_entries ) { return; }

            // One record per fetch block that ends in a branch, and the decode
            // stage consumes them in order; holding more than the queue does
            // would mean predicting branches the queue has no room to describe.
            if ( pending_.size() >= cfg_.ftq_entries ) { return; }

            const uint64_t pc       = ftq_pc_;
            const uint64_t line     = pc & line_mask_;
            const uint64_t line_end = line + cfg_.line_width;

            const VanadisFetchBlockPrediction p = bp_->predictFetchBlock(pc);

            if ( !p.found ) {
                // No branch marked at or after this address inside the block.
                // The block runs to the end of the line and the next one starts
                // at the next line -- the assumption a buffer-directed front
                // end makes, and the reason a straight-line region prefetches
                // perfectly with an empty buffer.
                push(pc, line_end, line);
                ftq_pc_ = line_end;
                continue;
            }

            if ( p.from_l2 && (cfg_.l2_bubble > 0) ) {
                bubble_ = cfg_.l2_bubble;
                if ( stats_.btb_bubbles ) { stats_.btb_bubbles->addData(cfg_.l2_bubble); }
            }

            if ( stats_.fetch_branches ) { stats_.fetch_branches->addData(1); }

            VanadisFetchStageBranch rec;
            rec.ckpt  = p.ckpt;
            rec.pc    = p.branch_pc;
            rec.next  = p.next;
            rec.taken = p.taken;
            pending_.push_back(rec);

            push(pc, p.branch_end, line);

            if ( 0 == p.next ) {
                // An indirect branch with no target yet. There is no honest
                // guess to make, so the run-ahead stops until the core
                // redirects it; running on would prefetch whatever address
                // zero happened to name.
                blocked_ = true;
                return;
            }

            ftq_pc_ = p.next;
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
    VanadisBranchUnit*        bp_     = nullptr;
    SST::Output*              output_ = nullptr;
    VanadisFDIPStats          stats_;

    uint64_t line_mask_ = ~63ull;
    uint64_t cycle_     = 0;

    std::deque<VanadisFTQEntry>         ftq_;
    std::deque<VanadisFetchStageBranch> pending_;
    uint64_t                            ftq_pc_  = 0;
    bool                                blocked_ = false;
    uint32_t                            bubble_  = 0;

    std::vector<FilterEntry> filter_;
    uint64_t                 filter_clock_ = 0;
};

} // namespace Vanadis
} // namespace SST

#endif
