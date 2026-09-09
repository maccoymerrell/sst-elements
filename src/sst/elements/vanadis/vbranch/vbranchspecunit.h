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

#ifndef _H_VANADIS_BRANCH_SPECULATIVE_UNIT
#define _H_VANADIS_BRANCH_SPECULATIVE_UNIT

#include "vbranch/vbranchbtb.h"
#include "vbranch/vbranchcheckpoint.h"
#include "vbranch/vbranchspeccore.h"
#include "vbranch/vbranchunit.h"

#include <sst/core/output.h>

#include <cstdint>
#include <vector>

namespace SST {
namespace Vanadis {

// The simulator's side of a direction predictor: the target buffer, the
// statistics, and the branch unit interface. The order in which a prediction,
// a training and a repair touch the predictor and its checkpoint ring is in
// VanadisSpeculativePredictor, which the standalone tests drive directly.
template <typename CORE>
class VanadisSpeculativeBranchUnit : public VanadisBranchUnit
{
public:
    using Checkpoint = typename CORE::Checkpoint;

    VanadisSpeculativeBranchUnit(ComponentId_t id, Params& params) :
        VanadisBranchUnit(id, params),
        btb(params.find<uint32_t>("branch_entries", 1536))
    {
        const uint32_t verbose = params.find<uint32_t>("verbose", 0);
        output_                = new SST::Output("[branch-unit]: ", verbose, 0, SST::Output::STDOUT);

        // Replaced by the core's own reorder-buffer depth at construction.
        predictor.setDepth(params.find<uint32_t>("checkpoint_entries", 512));

        stat_branch_hits          = registerStatistic<uint64_t>("branch_cache_hit", "1");
        stat_branch_misses        = registerStatistic<uint64_t>("branch_cache_miss", "1");
        stat_branch_cache_castout = registerStatistic<uint64_t>("branch_cache_castout", "1");

        stat_direction_mispredict = registerStatistic<uint64_t>("direction_mispredict", "1");
        stat_target_mispredict    = registerStatistic<uint64_t>("target_mispredict", "1");
        stat_ras_hit              = registerStatistic<uint64_t>("ras_hit", "1");
        stat_ras_miss             = registerStatistic<uint64_t>("ras_miss", "1");
        stat_checkpoints_repaired = registerStatistic<uint64_t>("checkpoints_repaired", "1");
        stat_wrong_path_branches  = registerStatistic<uint64_t>("wrong_path_branches", "1");
        stat_predictions          = registerStatistic<uint64_t>("predictions", "1");

        stat_class_branches[0] = registerStatistic<uint64_t>("conditional_branches", "1");
        stat_class_branches[1] = registerStatistic<uint64_t>("direct_jump_branches", "1");
        stat_class_branches[2] = registerStatistic<uint64_t>("direct_call_branches", "1");
        stat_class_branches[3] = registerStatistic<uint64_t>("indirect_jump_branches", "1");
        stat_class_branches[4] = registerStatistic<uint64_t>("indirect_call_branches", "1");
        stat_class_branches[5] = registerStatistic<uint64_t>("return_branches", "1");

        stat_class_mispredict[0] = registerStatistic<uint64_t>("conditional_mispredict", "1");
        stat_class_mispredict[1] = registerStatistic<uint64_t>("direct_jump_mispredict", "1");
        stat_class_mispredict[2] = registerStatistic<uint64_t>("direct_call_mispredict", "1");
        stat_class_mispredict[3] = registerStatistic<uint64_t>("indirect_jump_mispredict", "1");
        stat_class_mispredict[4] = registerStatistic<uint64_t>("indirect_call_mispredict", "1");
        stat_class_mispredict[5] = registerStatistic<uint64_t>("return_mispredict", "1");
    }

    virtual ~VanadisSpeculativeBranchUnit() { delete output_; }

    // ---- the address-only interface, still a target buffer -----------------
    void push(const uint64_t ins_addr, const uint64_t pred_addr) override
    {
        if ( btb.push(ins_addr, pred_addr) ) { stat_branch_cache_castout->addData(1); }
    }

    uint64_t predictAddress(const uint64_t addr) override { return btb.predict(addr); }

    bool contains(const uint64_t addr) override
    {
        const bool found = btb.contains(addr);
        if ( found ) { stat_branch_hits->addData(1); }
        else {
            stat_branch_misses->addData(1);
        }
        return found;
    }

    bool hasDirectionPrediction() const override { return true; }

    void setMaxInFlightBranches(uint32_t n) override { predictor.setDepth(n); }

    // ---- prediction --------------------------------------------------------
    bool predictDirection(
        uint64_t pc, VanadisBranchClass cls, uint64_t static_target, bool has_static_target, uint64_t fallthrough,
        bool predicted_taken_hint, VanadisBranchCheckpoint* ckpt) override
    {
        const bool dir = predictor.predictDirection(pc, cls, static_target, has_static_target, fallthrough, ckpt);

        if ( !ckpt->valid() ) {
            output_->fatal(
                CALL_INFO, -1,
                "Branch predictor checkpoint ring is exhausted (%" PRIu32 " entries, %" PRIu32
                " in flight). It is sized from the reorder buffer, so this cannot happen unless more "
                "branches are in flight than the reorder buffer holds.\n",
                predictor.capacity(), predictor.inFlight());
            return false;
        }

        stat_predictions->addData(1);
        stat_class_branches[static_cast<int>(cls)]->addData(1);

        return dir;
    }

    uint64_t predictTarget(
        uint64_t pc, VanadisBranchClass cls, uint64_t static_target, bool has_static_target, uint64_t fallthrough,
        VanadisBranchCheckpoint* ckpt) override
    {
        const bool     hit    = btb.contains(pc);
        const uint64_t target = hit ? btb.predict(pc) : 0;

        return predictor.predictTarget(pc, cls, static_target, has_static_target, fallthrough, target, hit, ckpt);
    }

    // ---- training ----------------------------------------------------------
    void update(
        uint64_t pc, VanadisBranchClass cls, bool taken, uint64_t target,
        const VanadisBranchCheckpoint& ckpt) override
    {
        if ( predictor.live(ckpt) ) {
            const Checkpoint& C = predictor.record(ckpt);

            if ( taken != C.pred_dir ) {
                stat_direction_mispredict->addData(1);
                stat_class_mispredict[static_cast<int>(cls)]->addData(1);
            }
            else if ( taken && (target != C.pred_target) ) {
                stat_target_mispredict->addData(1);
                stat_class_mispredict[static_cast<int>(cls)]->addData(1);
            }

            if ( C.used_ras ) {
                if ( target == C.pred_target ) { stat_ras_hit->addData(1); }
                else {
                    stat_ras_miss->addData(1);
                }
            }
        }

        if ( !predictor.update(pc, cls, taken, target, ckpt) ) {
            output_->fatal(
                CALL_INFO, -1,
                "Branch predictor checkpoint released out of order: branches retire in program order, "
                "so the record released has to be the oldest one in the ring.\n");
        }
    }

    // ---- repair ------------------------------------------------------------
    void repair(const VanadisBranchCheckpoint& ckpt, bool taken, uint64_t target) override
    {
        stat_wrong_path_branches->addData(predictor.repair(ckpt, taken));
        stat_checkpoints_repaired->addData(1);
    }

    void repairToCommit() override { predictor.repairToCommit(); }

    void serializeState(std::vector<uint8_t>& out) const override { predictor.serializeState(out); }

    void serializeSpeculativeState(std::vector<uint8_t>& out) const override
    {
        predictor.serializeSpeculativeState(out);
    }

protected:
    VanadisSpeculativePredictor<CORE> predictor;
    VanadisBranchTargetBuffer         btb;
    SST::Output*                      output_ = nullptr;

    Statistic<uint64_t>* stat_branch_hits;
    Statistic<uint64_t>* stat_branch_misses;
    Statistic<uint64_t>* stat_branch_cache_castout;
    Statistic<uint64_t>* stat_direction_mispredict;
    Statistic<uint64_t>* stat_target_mispredict;
    Statistic<uint64_t>* stat_ras_hit;
    Statistic<uint64_t>* stat_ras_miss;
    Statistic<uint64_t>* stat_checkpoints_repaired;
    Statistic<uint64_t>* stat_wrong_path_branches;
    Statistic<uint64_t>* stat_predictions;
    Statistic<uint64_t>* stat_class_branches[6];
    Statistic<uint64_t>* stat_class_mispredict[6];
};

// The statistics both direction predictors publish.
#define VANADIS_BRANCH_SPECULATIVE_STATISTICS                                                                    \
    { "branch_cache_hit", "Branch target buffer lookups that found an entry", "hits", 1 },                        \
        { "branch_cache_miss", "Branch target buffer lookups that found nothing", "misses", 1 },                  \
        { "branch_cache_castout", "Branch target buffer entries thrown out for capacity", "entries", 1 },         \
        { "predictions", "Branches predicted, correct path and wrong path alike", "branches", 1 },                \
        { "direction_mispredict", "Retired branches whose direction was predicted wrongly", "branches", 1 },      \
        { "target_mispredict", "Retired taken branches whose direction was right and target wrong", "branches",   \
          1 },                                                                                                   \
        { "ras_hit", "Returns whose target came from the return address stack and was right", "branches", 1 },    \
        { "ras_miss", "Returns whose target came from the return address stack and was wrong", "branches", 1 },   \
        { "checkpoints_repaired", "Squashes that restored the speculative history", "squashes", 1 },              \
        { "wrong_path_branches", "Branches discarded by a squash", "branches", 1 },                               \
        { "conditional_branches", "Conditional branches predicted", "branches", 1 },                              \
        { "direct_jump_branches", "Direct jumps predicted", "branches", 1 },                                      \
        { "direct_call_branches", "Direct calls predicted", "branches", 1 },                                      \
        { "indirect_jump_branches", "Indirect jumps predicted", "branches", 1 },                                  \
        { "indirect_call_branches", "Indirect calls predicted", "branches", 1 },                                  \
        { "return_branches", "Returns predicted", "branches", 1 },                                                \
        { "conditional_mispredict", "Conditional branches mispredicted", "branches", 1 },                         \
        { "direct_jump_mispredict", "Direct jumps mispredicted", "branches", 1 },                                 \
        { "direct_call_mispredict", "Direct calls mispredicted", "branches", 1 },                                 \
        { "indirect_jump_mispredict", "Indirect jumps mispredicted", "branches", 1 },                             \
        { "indirect_call_mispredict", "Indirect calls mispredicted", "branches", 1 },                             \
    {                                                                                                            \
        "return_mispredict", "Returns mispredicted", "branches", 1                                               \
    }

} // namespace Vanadis
} // namespace SST

#endif
