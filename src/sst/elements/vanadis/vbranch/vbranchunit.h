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

#ifndef _H_VANADIS_BRANCH_UNIT
#define _H_VANADIS_BRANCH_UNIT

#include <sst/core/subcomponent.h>

#include "inst/vspeculate.h"
#include "vbranch/vbranchcheckpoint.h"

#include <cstdint>
#include <list>
#include <unordered_map>
#include <vector>

namespace SST {
namespace Vanadis {

// What the fetch stage gets back from one branch target buffer lookup: where
// the block it was asked about ends, whether a branch ends it, and where fetch
// goes next. `next` of zero means the target is not known -- an indirect
// branch the buffer has never seen resolve -- and the run-ahead must stop
// rather than guess.
struct VanadisFetchBlockPrediction {
    VanadisBranchCheckpoint ckpt;
    uint64_t                branch_pc  = 0;
    uint64_t                branch_end = 0;
    uint64_t                next       = 0;
    VanadisBranchClass      cls        = VanadisBranchClass::CONDITIONAL;
    bool                    found      = false;   // a marked branch in this block
    bool                    block_hit  = false;   // the block had an entry at all
    bool                    from_l2    = false;
    bool                    taken      = false;
};

class VanadisBranchUnit : public SST::SubComponent {

public:
    SST_ELI_REGISTER_SUBCOMPONENT_API(SST::Vanadis::VanadisBranchUnit)

    SST_ELI_DOCUMENT_PARAMS()

    SST_ELI_DOCUMENT_STATISTICS()

    VanadisBranchUnit(ComponentId_t id, Params& params) : SubComponent(id) {}
    virtual ~VanadisBranchUnit() {}

    // ---- the address-only interface, which every unit still provides -------
    //
    // push() records at retire where a branch went, predictAddress() reads
    // that back and contains() says whether it is known. Together they are a
    // branch target buffer: a cache from branch address to target address.
    virtual void push(const uint64_t ins_addr, const uint64_t pred_addr) = 0;
    virtual uint64_t predictAddress(const uint64_t addr) = 0;
    virtual bool contains(const uint64_t addr) = 0;

    // ---- direction, target, training and repair ----------------------------

    // Does this unit predict the taken/not-taken bit? A unit that answers no
    // is a target buffer only, and the decoder keeps its address-only
    // behaviour for it.
    virtual bool hasDirectionPrediction() const { return false; }

    // At fetch. Takes a checkpoint, records into it everything the training at
    // retire will need, applies this branch's speculative history update using
    // the predicted direction and target, and returns the direction.
    //
    // An unconditional branch always returns true; the call is still made,
    // because every branch advances the history.
    virtual bool predictDirection(
        uint64_t pc, VanadisBranchClass cls, uint64_t static_target, bool has_static_target, uint64_t fallthrough,
        bool predicted_taken_hint, VanadisBranchCheckpoint* ckpt)
    {
        return false;
    }

    // At fetch, immediately after predictDirection. A return pops the return
    // address stack, a direct branch uses the target the decode already knows,
    // and only an indirect branch consults the target buffer. The chosen
    // target is written into the checkpoint so that repair can undo the stack
    // operation.
    virtual uint64_t predictTarget(
        uint64_t pc, VanadisBranchClass cls, uint64_t static_target, bool has_static_target, uint64_t fallthrough,
        VanadisBranchCheckpoint* ckpt)
    {
        return 0;
    }

    // At retire, in program order. Trains the tables from what the checkpoint
    // recorded, advances the architected copy of the history and releases the
    // checkpoint.
    // `width` is the branch's own width in bytes, which the buffer needs to
    // mark a branch it has never held, and which a record-less branch -- one
    // the buffer never marked, so the fetch stage never predicted -- has
    // nowhere else to come from.
    virtual void update(
        uint64_t pc, VanadisBranchClass cls, bool taken, uint64_t target, const VanadisBranchCheckpoint& ckpt,
        uint8_t width)
    {}

    // On a branch misprediction, after update() for the same branch. Puts the
    // speculative histories, the return address stack and the loop
    // bookkeeping back to what the checkpoint holds, then re-applies this
    // branch's own history update with the direction and target that actually
    // happened. Every younger checkpoint is released.
    virtual void repair(const VanadisBranchCheckpoint& ckpt, bool taken, uint64_t target) {}

    // On a redirect that is not a branch misprediction: thread start, and the
    // paths that resume a thread at a new instruction pointer after a system
    // call or a fault. Everything unretired is being thrown away, so the
    // correct speculative state is the architected state.
    virtual void repairToCommit() {}

    // The checkpoint ring is sized from the reorder buffer, so that a branch
    // that holds a reorder-buffer entry always has a checkpoint to hold.
    virtual void setMaxInFlightBranches(uint32_t n) {}

    // A byte image of everything the predictor holds. Used by the trace-driven
    // tests to compare two predictors exactly.
    virtual void serializeState(std::vector<uint8_t>& out) const { out.clear(); }

    // A byte image of the speculative history alone, which is what a repair
    // has to put back.
    virtual void serializeSpeculativeState(std::vector<uint8_t>& out) const { out.clear(); }

    // ---- the two-stage front end -------------------------------------------
    //
    // A unit that answers yes here carries a fetch-stage branch target buffer
    // and steers fetch from it, a cycle or more before the branch is decoded.
    // The decode stage then overrides with the tagged predictor.
    virtual bool hasFetchStage() const { return false; }

    // The aligned instruction fetch block the buffer is indexed by.
    virtual uint64_t fetchBlockBytes() const { return 64; }

    // STAGE ONE. One buffer lookup for the block `pc` falls in.
    virtual VanadisFetchBlockPrediction predictFetchBlock(uint64_t pc)
    {
        return VanadisFetchBlockPrediction();
    }

    // A branch the buffer does not mark still takes a record, so that the ring
    // stays in fetch order. It predicts nothing and moves no history.
    virtual bool recordUnpredicted(
        uint64_t pc, VanadisBranchClass cls, uint64_t fallthrough, uint8_t width, VanadisBranchCheckpoint* ckpt)
    {
        return false;
    }

    // Does the buffer mark a branch at this address? A branch it does not mark
    // is not a branch as far as the fetch stage is concerned: it is predicted
    // not-taken, it enters no history and it costs nothing.
    virtual bool btbMarks(uint64_t pc) { return false; }

    // STAGE ONE, late. The decode stage reached a marked branch the fetch
    // stage has not predicted, because the run-ahead is elsewhere. This makes
    // the record the fetch stage would have made, from the same buffer entry.
    virtual bool predictAtDecode(
        uint64_t pc, VanadisBranchClass cls, uint64_t static_target, bool has_static_target, uint64_t fallthrough,
        uint8_t width, VanadisBranchCheckpoint* ckpt, uint64_t* next_pc)
    {
        return false;
    }

    // STAGE TWO. Returns true when the tagged predictor disagreed with the
    // buffer and fetch has to be re-steered to *next_pc.
    virtual bool overrideAtDecode(
        const VanadisBranchCheckpoint& ckpt, uint64_t static_target, bool has_static_target, uint64_t fallthrough,
        uint64_t* next_pc)
    {
        return false;
    }

    // The decode stage consumed a record without overriding.
    virtual void markDecoded(const VanadisBranchCheckpoint& ckpt) {}

    // Throw away the records of every branch the fetch stage predicted that
    // the decode stage has not reached. Returns how many.
    virtual uint32_t discardRunAhead() { return 0; }

    // A branch resolved wrongly and is being repaired while it is still in the
    // reorder buffer, at execute rather than at retire. Returns how many
    // wrong-path branch records went with it.
    virtual uint32_t repairAtExecute(
        const VanadisBranchCheckpoint& ckpt, uint64_t pc, VanadisBranchClass cls, bool taken, uint64_t target)
    {
        return 0;
    }
};

} // namespace Vanadis
} // namespace SST

#endif
