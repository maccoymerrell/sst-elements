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
    virtual void update(
        uint64_t pc, VanadisBranchClass cls, bool taken, uint64_t target, const VanadisBranchCheckpoint& ckpt)
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
};

} // namespace Vanadis
} // namespace SST

#endif
