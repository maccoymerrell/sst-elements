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

#ifndef _H_VANADIS_BRANCH_SPECULATIVE_CORE
#define _H_VANADIS_BRANCH_SPECULATIVE_CORE

#include "vbranch/vbranchcheckpoint.h"

#include <cstdint>
#include <vector>

namespace SST {
namespace Vanadis {

// The order in which a prediction, a training and a repair touch a direction
// predictor and its checkpoint ring. It is separate from the simulator's
// subcomponent, and depends on nothing in the simulator, so that the same code
// can be driven from a recorded branch trace by the standalone tests.
//
// The predictor itself is the template argument and has to provide
//
//   Checkpoint            a per-branch record
//   snapshot(C, pc)       write into C what a repair will need to put back
//   predict(C, pc, cls, hist_target, fallthrough) -> direction, and advance
//                         the speculative history
//   update(pc, cls, taken, C)   train, and advance the architected history
//   restoreWords(C)       put back only the per-index history words C moved
//   repair(C, taken)      put back the rest, then replay C with `taken`
//   repairToCommit()      make the speculative history the architected one
//   rasPush(addr) / rasPop()
//   serializeAll(out) / serializeSpeculative(out)
//
template <typename CORE>
class VanadisSpeculativePredictor
{
public:
    using Checkpoint = typename CORE::Checkpoint;

    VanadisSpeculativePredictor() { ring_.resize(512); }

    void setDepth(const uint32_t n) { ring_.resize(n + 8); }

    CORE&       predictor() { return core_; }
    const CORE& predictor() const { return core_; }

    uint32_t capacity() const { return ring_.capacity(); }
    uint32_t inFlight() const { return ring_.inFlight(); }
    uint32_t tailSlot() const { return ring_.tailSlot(); }
    uint64_t tailPC() const
    {
        VanadisBranchCheckpoint h;
        h.slot = (uint16_t)ring_.tailSlot();
        h.gen  = 0;
        return const_cast<VanadisSpeculativePredictor*>(this)->ring_.slotRecord(ring_.tailSlot()).pc;
    }
    uint32_t headSlot() const { return ring_.headSlot(); }
    uint64_t exhaustedCount() const { return ring_.exhaustedCount(); }
    uint64_t staleCount() const { return ring_.staleCount(); }

    bool live(const VanadisBranchCheckpoint& h) const { return ring_.handleIsLive(h); }
    Checkpoint&       record(const VanadisBranchCheckpoint& h) { return ring_.record(h); }
    const Checkpoint& record(const VanadisBranchCheckpoint& h) const
    {
        return const_cast<VanadisSpeculativePredictor*>(this)->ring_.record(h);
    }

    // At fetch. Takes a record, notes in it what a repair will need, and
    // advances the speculative history with the direction it returns.
    // Returns false with an invalid handle if the ring is full, which cannot
    // happen when the ring is sized from the reorder buffer.
    bool predictDirection(
        const uint64_t pc, const VanadisBranchClass cls, const uint64_t static_target, const bool has_static_target,
        const uint64_t fallthrough, VanadisBranchCheckpoint* handle)
    {
        Checkpoint* C = ring_.allocate(handle);
        if ( nullptr == C ) { return false; }

        core_.snapshot(*C, pc);
        C->single_stage = true;

        // The history advances with the branch's own target. For a
        // conditional branch the decode already knows it; for every other
        // class nothing in the history update reads it.
        const uint64_t hist_target = has_static_target ? static_target : fallthrough;

        core_.predict(*C, pc, cls, hist_target, fallthrough);

        // NO FETCH STAGE: there is one prediction and it is the tagged one, so
        // the two answers the checkpoint holds are made the same and the
        // override at decode has nothing to do.
        C->fetch_dir   = C->tage_dir;
        C->pred_dir    = C->tage_dir;
        C->final_dir   = C->tage_dir;
        C->pred_target = fallthrough;
        C->used_ras    = false;

        return C->tage_dir;
    }

    // At fetch, after predictDirection, for a branch predicted taken.
    // `btb_target` is what the target buffer holds for this address, and
    // `btb_hit` whether it holds anything; only an indirect branch uses them.
    uint64_t predictTarget(
        const uint64_t pc, const VanadisBranchClass cls, const uint64_t static_target, const bool has_static_target,
        const uint64_t fallthrough, const uint64_t btb_target, const bool btb_hit, VanadisBranchCheckpoint* handle)
    {
        if ( !ring_.handleIsLive(*handle) ) { return fallthrough; }

        Checkpoint& C    = ring_.record(*handle);
        uint64_t    next = fallthrough;

        switch ( cls ) {
        case VanadisBranchClass::CONDITIONAL:
        case VanadisBranchClass::DIRECT_JUMP:
        case VanadisBranchClass::DIRECT_CALL:
            next = has_static_target ? static_target : fallthrough;
            break;

        case VanadisBranchClass::INDIRECT_JUMP:
        case VanadisBranchClass::INDIRECT_CALL:
            next = btb_hit ? btb_target : fallthrough;
            break;

        case VanadisBranchClass::RETURN:
        {
            const uint64_t from_stack = core_.rasPop();
            C.used_ras                = true;
            next                      = (0 == from_stack) ? (btb_hit ? btb_target : fallthrough) : from_stack;
        } break;
        }

        // A call's return address goes on the stack now, speculatively, and
        // comes back off it if this path turns out to be wrong.
        if ( vanadisBranchIsCall(cls) ) { core_.rasPush(fallthrough); }

        C.pred_target = next;
        return next;
    }

    // ---- THE TWO-STAGE FRONT END ------------------------------------------

    // STAGE ONE, AT FETCH. One branch target buffer lookup has already been
    // made by the caller and its slot is passed in: the branch's class, width
    // and target, and the two bits of the bimodal counter that live in that
    // slot and are this predictor's base prediction. This takes a record,
    // notes in it what a repair will need, makes both stages' predictions
    // against the same history, advances the speculative history with the
    // FETCH STAGE's answer, and returns the address fetch should go to next.
    //
    // A returned next address of zero means the run-ahead cannot continue: an
    // indirect branch whose target nothing knows yet.
    bool predictAtFetch(
        const uint64_t pc, const VanadisBranchClass cls, const uint64_t static_target, const bool has_static_target,
        const uint64_t fallthrough, const bool bim_valid, const int8_t bim_pred, const int8_t bim_hyst,
        const bool btb_block_hit, const bool btb_from_l2, const uint8_t width, const uint64_t btb_target,
        const bool btb_has_target, VanadisBranchCheckpoint* handle, uint64_t* next_pc)
    {
        *next_pc = fallthrough;

        Checkpoint* C = ring_.allocate(handle);
        if ( nullptr == C ) { return false; }

        core_.snapshot(*C, pc);

        C->bim_valid     = bim_valid;
        C->bim_pred      = bim_pred;
        C->bim_hyst      = bim_hyst;
        C->bim_dirty     = false;
        C->btb_block_hit = btb_block_hit;
        C->btb_from_l2   = btb_from_l2;
        C->width         = width;
        C->overridden    = false;

        const uint64_t hist_target = has_static_target ? static_target : fallthrough;
        const bool     dir         = core_.predict(*C, pc, cls, hist_target, fallthrough);

        C->used_ras = false;

        uint64_t next = fallthrough;

        if ( dir ) {
            switch ( cls ) {
            case VanadisBranchClass::CONDITIONAL:
            case VanadisBranchClass::DIRECT_JUMP:
            case VanadisBranchClass::DIRECT_CALL:
                next = has_static_target ? static_target : (btb_has_target ? btb_target : 0);
                break;

            case VanadisBranchClass::INDIRECT_JUMP:
            case VanadisBranchClass::INDIRECT_CALL:
                next = btb_has_target ? btb_target : 0;
                break;

            case VanadisBranchClass::RETURN:
            {
                const uint64_t from_stack = core_.rasPop();
                C->used_ras               = true;
                next                      = (0 == from_stack) ? (btb_has_target ? btb_target : 0) : from_stack;
            } break;
            }
        }

        if ( vanadisBranchIsCall(cls) ) { core_.rasPush(fallthrough); }

        C->pred_target  = next;
        C->fetch_target = next;
        C->final_dir    = dir;
        C->final_target = next;

        *next_pc = next;
        return dir;
    }

    // A BRANCH THE BUFFER NEVER MARKED. The fetch stage ran through its
    // address as straight-line code, so it was implicitly predicted not-taken
    // and no history moved for it. A record is still taken, and taken HERE, in
    // decode order -- which is the same as fetch order for a branch the fetch
    // stage never predicted -- because a squash has to be able to say "every
    // record younger than this branch", and a branch with no record has no
    // place in that ordering.
    bool recordUnpredicted(
        const uint64_t pc, const VanadisBranchClass cls, const uint64_t fallthrough, const uint8_t width,
        VanadisBranchCheckpoint* handle)
    {
        Checkpoint* C = ring_.allocate(handle);
        if ( nullptr == C ) { return false; }

        core_.snapshot(*C, pc);

        C->cls          = cls;
        C->hist_target  = fallthrough;
        C->fallthrough  = fallthrough;
        C->no_history   = true;
        C->bim_valid    = false;
        C->bim_dirty    = false;
        C->btb_block_hit = false;
        C->btb_from_l2   = false;
        C->width        = width;
        C->overridden   = false;
        C->used_ras     = false;
        C->fetch_dir    = false;
        C->tage_dir     = false;
        C->pred_dir     = false;
        C->pred_target  = fallthrough;
        C->fetch_target = fallthrough;
        C->final_dir    = false;
        C->final_target = fallthrough;
        return true;
    }

    // STAGE TWO, AT DECODE. The branch has been decoded, so its class and, for
    // a direct branch, its target are exact. The tagged tables, statistical
    // corrector and loop predictor answered when the record was made; if that
    // answer differs from the one the fetch stage steered with, the fetch
    // stage was wrong about a path the machine has since been fetching, and
    // this puts the speculative history back, throws away the run-ahead's
    // records and hands back the address fetch should be re-steered to.
    //
    // Returns true when it overrode.
    bool overrideAtDecode(
        const VanadisBranchCheckpoint& handle, const uint64_t static_target, const bool has_static_target,
        const uint64_t fallthrough, uint64_t* next_pc, uint32_t* discarded)
    {
        *discarded = 0;

        if ( !ring_.handleIsLive(handle) ) { return false; }

        Checkpoint& C = ring_.record(handle);

        if ( C.tage_dir == C.fetch_dir ) {
            ring_.markDecoded(handle);
            *next_pc = C.pred_target;
            return false;
        }

        // Every record the fetch stage made after this branch moved the
        // per-index history words; putting them back newest first leaves each
        // at the value this branch found.
        const uint32_t younger = ring_.countYoungerThan(handle);
        for ( uint32_t i = 0; i < younger; ++i ) {
            core_.restoreWords(ring_.younger(i));
        }

        core_.applyOverride(C, C.tage_dir);

        uint64_t next = fallthrough;
        if ( C.tage_dir ) {
            next = has_static_target ? static_target : C.fetch_target;
            if ( 0 == next ) { next = fallthrough; }
        }

        C.pred_target  = next;
        C.final_dir    = C.tage_dir;
        C.final_target = next;

        *discarded = ring_.discardYoungerThan(handle);
        ring_.markDecoded(handle);

        *next_pc = next;
        return true;
    }

    // The decode stage consumed this record without overriding.
    void markDecoded(const VanadisBranchCheckpoint& handle) { ring_.markDecoded(handle); }

    // The fetch stage's run-ahead is on a path the decode stage is not taking.
    // Its records go; nothing already decoded is touched.
    uint32_t discardRunAhead()
    {
        const uint32_t undecoded = ring_.undecodedCount();
        if ( 0 == undecoded ) { return 0; }

        for ( uint32_t i = 0; i < undecoded; ++i ) {
            core_.restoreWords(ring_.younger(i));
        }

        // `younger(undecoded - 1)` is the oldest record the fetch stage made
        // that the decode stage never consumed, and its snapshot is the state
        // the machine was in before the run-ahead began.
        core_.restoreOnly(ring_.younger(undecoded - 1));

        return ring_.discardUndecoded();
    }

    // A branch resolved wrongly and is being repaired AT EXECUTE, while it is
    // still in the reorder buffer. Everything younger goes; this branch's own
    // record stays, because it has still to retire and train the tables.
    uint32_t repairAtExecute(
        const VanadisBranchCheckpoint& handle, const uint64_t pc, const VanadisBranchClass cls, const bool taken,
        bool* bim_dirty, int8_t* bim_pred, int8_t* bim_hyst)
    {
        *bim_dirty = false;

        if ( !ring_.handleIsLive(handle) ) { return 0; }

        // THE TABLES LEARN HERE. The machine is about to fetch the corrected
        // path and, in a loop, that path begins with this same branch; the
        // tables have to hold the answer that was just proved before it is
        // predicted again.
        Checkpoint& C = ring_.record(handle);
        core_.trainAtResolve(pc, cls, taken, C);
        *bim_dirty = C.bim_dirty;
        *bim_pred  = C.bim_pred;
        *bim_hyst  = C.bim_hyst;
        C.bim_dirty = false;

        const uint32_t wrong_path = ring_.countYoungerThan(handle);
        for ( uint32_t i = 0; i < wrong_path; ++i ) {
            core_.restoreWords(ring_.younger(i));
        }

        core_.repair(ring_.record(handle), taken);
        ring_.discardYoungerThan(handle);
        return wrong_path;
    }

    const Checkpoint* liveRecord(const VanadisBranchCheckpoint& handle) const
    {
        if ( !ring_.handleIsLive(handle) ) { return nullptr; }
        return &const_cast<VanadisSpeculativePredictor*>(this)->ring_.record(handle);
    }

    // At retire, in program order. Returns false if the record released was
    // not the oldest one, which branches retiring in order makes impossible.
    bool update(const uint64_t pc, const VanadisBranchClass cls, const bool taken, const uint64_t target,
                const VanadisBranchCheckpoint& handle)
    {
        if ( !ring_.handleIsLive(handle) ) { return true; }

        Checkpoint& C = ring_.record(handle);
        core_.update(pc, cls, taken, C);
        return ring_.retire(handle);
    }

    // On a misprediction, after update() for the same branch. Returns how many
    // wrong-path branches were discarded.
    uint32_t repair(const VanadisBranchCheckpoint& handle, const bool taken)
    {
        if ( !handle.valid() || (handle.slot >= ring_.capacity()) ) { return 0; }

        // Everything left in the ring was fetched after this branch, because
        // this branch's own record has already been released. Putting the
        // per-index history words back newest first leaves every one of them
        // at the value this branch found.
        const uint32_t wrong_path = ring_.youngerCount();

        for ( uint32_t i = 0; i < wrong_path; ++i ) {
            core_.restoreWords(ring_.younger(i));
        }

        core_.repair(ring_.record(handle), taken);

        ring_.discardAll();
        return wrong_path;
    }

    void repairToCommit()
    {
        ring_.discardAll();
        core_.repairToCommit();
    }

    void serializeState(std::vector<uint8_t>& out) const { core_.serializeAll(out); }
    void serializeSpeculativeState(std::vector<uint8_t>& out) const { core_.serializeSpeculative(out); }

private:
    CORE                              core_;
    VanadisCheckpointRing<Checkpoint> ring_;
};

} // namespace Vanadis
} // namespace SST

#endif
