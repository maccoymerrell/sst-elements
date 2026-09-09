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

        // The history advances with the branch's own target. For a
        // conditional branch the decode already knows it; for every other
        // class nothing in the history update reads it.
        const uint64_t hist_target = has_static_target ? static_target : fallthrough;

        const bool dir = core_.predict(*C, pc, cls, hist_target, fallthrough);

        C->pred_target = fallthrough;
        C->used_ras    = false;

        return dir;
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
