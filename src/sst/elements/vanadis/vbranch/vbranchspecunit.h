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

#include "vbranch/vbranchfetchbtb.h"
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
        btb(params.find<uint32_t>("btb_l1_entries", 1536), params.find<uint32_t>("btb_l2_entries", 7680),
            params.find<uint64_t>("btb_block_bytes", 64))
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

        stat_btb_l1_hit        = registerStatistic<uint64_t>("btb_l1_hit", "1");
        stat_btb_l2_hit        = registerStatistic<uint64_t>("btb_l2_hit", "1");
        stat_btb_miss          = registerStatistic<uint64_t>("btb_miss", "1");
        stat_btb_alloc         = registerStatistic<uint64_t>("btb_alloc", "1");
        stat_btb_evict         = registerStatistic<uint64_t>("btb_evict", "1");
        stat_btb_slot_overflow = registerStatistic<uint64_t>("btb_slot_overflow", "1");
        stat_btb_unknown       = registerStatistic<uint64_t>("btb_unknown_at_decode", "1");
        stat_fetch_mispredict  = registerStatistic<uint64_t>("fetch_stage_mispredict", "1");
        stat_override          = registerStatistic<uint64_t>("override_count", "1");
        stat_override_right    = registerStatistic<uint64_t>("override_correct", "1");
        stat_override_wrong    = registerStatistic<uint64_t>("override_wrong", "1");
        stat_override_missing  = registerStatistic<uint64_t>("override_missed", "1");
        stat_runahead_discard  = registerStatistic<uint64_t>("runahead_records_discarded", "1");
        stat_execute_repairs   = registerStatistic<uint64_t>("execute_repairs", "1");

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
    // THE ADDRESS-ONLY INTERFACE IS THE FETCH BUFFER, READ BY ITS OWN NAME.
    // There is one target buffer in this unit, not two: these three methods
    // are the old address-only view of the same slots the fetch stage reads.
    void push(const uint64_t ins_addr, const uint64_t pred_addr) override { btb.setTarget(ins_addr, pred_addr); }

    uint64_t predictAddress(const uint64_t addr) override
    {
        VanadisFetchBTB::Slot* s = btb.slotFor(addr);
        return ((nullptr != s) && s->has_target) ? s->target : 0;
    }

    bool contains(const uint64_t addr) override
    {
        VanadisFetchBTB::Slot* s     = btb.slotFor(addr);
        const bool             found = (nullptr != s) && s->has_target;
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
        VanadisFetchBTB::Slot* s      = btb.slotFor(pc);
        const bool             hit    = (nullptr != s) && s->has_target;
        const uint64_t         target = hit ? s->target : 0;

        return predictor.predictTarget(pc, cls, static_target, has_static_target, fallthrough, target, hit, ckpt);
    }

    // ---- the two-stage front end -------------------------------------------

    bool     hasFetchStage() const override { return true; }
    uint64_t fetchBlockBytes() const override { return btb.blockBytes(); }

    VanadisFetchBlockPrediction predictFetchBlock(uint64_t pc) override
    {
        VanadisFetchBlockPrediction out;

        const VanadisFetchBTB::Lookup lk = btb.lookup(pc);
        out.block_hit = lk.block_hit;
        out.from_l2   = lk.from_l2;

        if ( lk.from_l2 ) { stat_btb_l2_hit->addData(1); }
        else if ( lk.block_hit ) {
            stat_btb_l1_hit->addData(1);
        }
        else {
            stat_btb_miss->addData(1);
        }

        if ( !lk.found ) { return out; }

        out.found      = true;
        out.branch_pc  = lk.branch_pc;
        out.branch_end = lk.branch_pc + lk.slot.width;
        out.cls        = lk.slot.cls;

        uint64_t next = 0;
        out.taken     = predictor.predictAtFetch(
            lk.branch_pc, lk.slot.cls, lk.slot.target, lk.slot.has_target && !vanadisBranchIsIndirect(lk.slot.cls),
            out.branch_end, true, lk.slot.pred, lk.slot.hyst, lk.block_hit, lk.from_l2, lk.slot.width, lk.slot.target,
            lk.slot.has_target, &out.ckpt, &next);

        if ( !out.ckpt.valid() ) {
            output_->fatal(
                CALL_INFO, -1,
                "Branch predictor checkpoint ring is exhausted (%" PRIu32 " entries, %" PRIu32 " in flight).\n",
                predictor.capacity(), predictor.inFlight());
            out.found = false;
            return out;
        }

        stat_predictions->addData(1);
        stat_class_branches[static_cast<int>(lk.slot.cls)]->addData(1);

        out.next = out.taken ? next : out.branch_end;
        return out;
    }

    bool btbMarks(uint64_t pc) override { return nullptr != btb.slotFor(pc); }

    bool recordUnpredicted(
        uint64_t pc, VanadisBranchClass cls, uint64_t fallthrough, uint8_t width,
        VanadisBranchCheckpoint* ckpt) override
    {
        if ( !predictor.recordUnpredicted(pc, cls, fallthrough, width, ckpt) ) {
            output_->fatal(
                CALL_INFO, -1,
                "Branch predictor checkpoint ring is exhausted (%" PRIu32 " entries, %" PRIu32 " in flight).\n",
                predictor.capacity(), predictor.inFlight());
            return false;
        }

        stat_predictions->addData(1);
        stat_class_branches[static_cast<int>(cls)]->addData(1);
        return true;
    }

    bool predictAtDecode(
        uint64_t pc, VanadisBranchClass cls, uint64_t static_target, bool has_static_target, uint64_t fallthrough,
        uint8_t width, VanadisBranchCheckpoint* ckpt, uint64_t* next_pc) override
    {
        stat_btb_unknown->addData(1);

        // The branch IS marked -- the decode stage checked before calling --
        // but the fetch stage's run-ahead is somewhere else, so the prediction
        // the fetch stage would have made is made here instead, out of the
        // same entry.
        VanadisFetchBTB::Slot* s          = btb.slotFor(pc);
        const bool             marked     = (nullptr != s);
        const bool             has_target = marked && s->has_target;
        const uint64_t         target     = has_target ? s->target : 0;

        uint64_t next = fallthrough;
        predictor.predictAtFetch(
            pc, cls, static_target, has_static_target, fallthrough, marked, marked ? s->pred : 0,
            marked ? s->hyst : 1, marked, false, width, target, has_target, ckpt, &next);

        if ( !ckpt->valid() ) {
            output_->fatal(
                CALL_INFO, -1,
                "Branch predictor checkpoint ring is exhausted (%" PRIu32 " entries, %" PRIu32 " in flight).\n",
                predictor.capacity(), predictor.inFlight());
            return false;
        }

        stat_predictions->addData(1);
        stat_class_branches[static_cast<int>(cls)]->addData(1);

        *next_pc = next;
        return true;
    }

    bool overrideAtDecode(
        const VanadisBranchCheckpoint& ckpt, uint64_t static_target, bool has_static_target, uint64_t fallthrough,
        uint64_t* next_pc) override
    {
        uint32_t   discarded = 0;
        const bool did       = predictor.overrideAtDecode(ckpt, static_target, has_static_target, fallthrough,
                                                          next_pc, &discarded);
        if ( did ) {
            stat_override->addData(1);
            if ( discarded > 0 ) { stat_runahead_discard->addData(discarded); }
        }
        return did;
    }

    void markDecoded(const VanadisBranchCheckpoint& ckpt) override { predictor.markDecoded(ckpt); }

    uint32_t discardRunAhead() override
    {
        const uint32_t n = predictor.discardRunAhead();
        if ( n > 0 ) { stat_runahead_discard->addData(n); }
        return n;
    }

    uint32_t repairAtExecute(
        const VanadisBranchCheckpoint& ckpt, uint64_t pc, VanadisBranchClass cls, bool taken,
        uint64_t target) override
    {
        bool   bim_dirty = false;
        int8_t bim_pred  = 0;
        int8_t bim_hyst  = 1;

        const uint32_t n = predictor.repairAtExecute(ckpt, pc, cls, taken, &bim_dirty, &bim_pred, &bim_hyst);

        // The buffer's own counter learns here too, for the same reason.
        if ( bim_dirty ) { btb.setBimodal(pc, bim_pred, bim_hyst); }
        if ( taken ) { btb.setTarget(pc, target); }

        stat_execute_repairs->addData(1);
        stat_wrong_path_branches->addData(n);
        return n;
    }

    // ---- training ----------------------------------------------------------
    void update(
        uint64_t pc, VanadisBranchClass cls, bool taken, uint64_t target, const VanadisBranchCheckpoint& ckpt,
        uint8_t ins_width) override
    {
        uint8_t  width          = ins_width;
        bool     marked         = false;
        bool     bim_dirty      = false;
        int8_t   bim_pred       = 0;
        int8_t   bim_hyst       = 1;

        if ( predictor.live(ckpt) ) {
            const Checkpoint& C = predictor.record(ckpt);
            width  = C.width;
            marked = C.bim_valid;

            // THE TWO STAGES ARE SCORED SEPARATELY, IN PROGRAM ORDER.
            //
            // `fetch_dir` is what the branch target buffer's bimodal counter
            // steered fetch with; `final_dir` is what the front end settled on
            // after the tagged predictor had its say. Repair never writes
            // either, so both are still what they were when the branch was
            // predicted.
            if ( vanadisBranchIsConditional(cls) && (taken != C.fetch_dir) ) {
                stat_fetch_mispredict->addData(1);
            }

            if ( C.overridden ) {
                if ( taken == C.final_dir ) { stat_override_right->addData(1); }
                else {
                    stat_override_wrong->addData(1);
                }
            }
            else if ( vanadisBranchIsConditional(cls) && (taken != C.final_dir) && (C.tage_dir == taken) ) {
                // The tagged predictor had the right answer and did not use it,
                // which cannot happen while the override is unconditional; the
                // counter exists so that it is visible if it ever does.
                stat_override_missing->addData(1);
            }

            if ( taken != C.final_dir ) {
                stat_direction_mispredict->addData(1);
                stat_class_mispredict[static_cast<int>(cls)]->addData(1);
            }
            else if ( taken && (target != C.final_target) ) {
                stat_target_mispredict->addData(1);
                stat_class_mispredict[static_cast<int>(cls)]->addData(1);
            }

            if ( C.used_ras ) {
                if ( target == C.final_target ) { stat_ras_hit->addData(1); }
                else {
                    stat_ras_miss->addData(1);
                }
            }
        }

        if ( !predictor.update(pc, cls, taken, target, ckpt) ) {
            output_->fatal(
                CALL_INFO, -1,
                "Branch predictor checkpoint released out of order at pc 0x%" PRIx64 ": slot %" PRIu16
                " gen %" PRIu16 ", ring tail %" PRIu32 " (pc 0x%" PRIx64 ") head %" PRIu32 " in-flight %" PRIu32
                " live %d.\n",
                pc, ckpt.slot, ckpt.gen, predictor.tailSlot(), predictor.tailPC(), predictor.headSlot(), predictor.inFlight(),
                predictor.live(ckpt) ? 1 : 0);
        }

        // THE BUFFER LEARNS HERE, AND ONLY HERE. A branch it did not hold is
        // marked now that it has resolved -- a conditional only if it resolved
        // taken, which is the rule the marked slots exist to express -- and a
        // branch it did hold has its bimodal counter written back and, if it
        // is indirect, its target refreshed.
        {
            const Checkpoint* C = predictor.liveRecord(ckpt);
            if ( nullptr != C ) {
                bim_dirty = C->bim_dirty;
                bim_pred  = C->bim_pred;
                bim_hyst  = C->bim_hyst;
            }
        }

        syncBufferCounters();

        if ( !marked ) {
            const uint64_t alloc_target = taken ? target : 0;
            const size_t   before       = (size_t)btb.allocations();
            btb.allocate(pc, cls, width, alloc_target, taken, taken);
            if ( (size_t)btb.allocations() != before ) { stat_btb_alloc->addData(1); }
        }
        else {
            if ( bim_dirty ) { btb.setBimodal(pc, bim_pred, bim_hyst); }
            if ( taken ) { btb.setTarget(pc, target); }
        }
    }

    // ---- repair ------------------------------------------------------------
    void repair(const VanadisBranchCheckpoint& ckpt, bool taken, uint64_t target) override
    {
        // A branch the buffer never marked made no record, so there is nothing
        // to repair to: everything unretired is being thrown away, and the
        // architected state is the right speculative state.
        if ( !predictor.live(ckpt) ) {
            predictor.repairToCommit();
            stat_checkpoints_repaired->addData(1);
            return;
        }

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
    // The buffer keeps plain counters; this hands their increments to the
    // statistics engine once per retired branch, which is often enough for
    // structures that only change when a branch resolves.
    void syncBufferCounters()
    {
        const uint64_t evict = btb.l1Evictions() + btb.l2Evictions();
        if ( evict > last_evict_ ) {
            stat_btb_evict->addData(evict - last_evict_);
            last_evict_ = evict;
        }
        const uint64_t over = btb.slotOverflow();
        if ( over > last_overflow_ ) {
            stat_btb_slot_overflow->addData(over - last_overflow_);
            last_overflow_ = over;
        }
    }

    uint64_t last_evict_    = 0;
    uint64_t last_overflow_ = 0;

    VanadisSpeculativePredictor<CORE> predictor;
    VanadisFetchBTB                   btb;
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

    Statistic<uint64_t>* stat_btb_l1_hit;
    Statistic<uint64_t>* stat_btb_l2_hit;
    Statistic<uint64_t>* stat_btb_miss;
    Statistic<uint64_t>* stat_btb_alloc;
    Statistic<uint64_t>* stat_btb_evict;
    Statistic<uint64_t>* stat_btb_slot_overflow;
    Statistic<uint64_t>* stat_btb_unknown;
    Statistic<uint64_t>* stat_fetch_mispredict;
    Statistic<uint64_t>* stat_override;
    Statistic<uint64_t>* stat_override_right;
    Statistic<uint64_t>* stat_override_wrong;
    Statistic<uint64_t>* stat_override_missing;
    Statistic<uint64_t>* stat_runahead_discard;
    Statistic<uint64_t>* stat_execute_repairs;
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
        { "return_mispredict", "Returns mispredicted", "branches", 1 },                                          \
        { "btb_l1_hit", "Fetch-stage lookups answered by the first-level branch target buffer", "lookups", 1 },  \
        { "btb_l2_hit", "Fetch-stage lookups answered by the second-level buffer", "lookups", 1 },               \
        { "btb_miss", "Fetch-stage lookups that found no entry for the block", "lookups", 1 },                    \
        { "btb_alloc", "Branches marked in the buffer when they resolved", "branches", 1 },                       \
        { "btb_evict", "Buffer entries thrown out", "entries", 1 },                                               \
        { "btb_slot_overflow", "Times a block held more branches than one entry describes", "events", 1 },        \
        { "btb_unknown_at_decode", "Branches the decode stage met that the buffer had not marked", "branches",    \
          1 },                                                                                                   \
        { "fetch_stage_mispredict", "Conditional branches the buffer's bimodal counter got wrong", "branches",    \
          1 },                                                                                                   \
        { "override_count", "Times the tagged predictor overrode the fetch stage", "branches", 1 },               \
        { "override_correct", "Of those, the ones the tagged predictor got right", "branches", 1 },               \
        { "override_wrong", "Of those, the ones it got wrong", "branches", 1 },                                   \
        { "override_missed", "Branches the tagged predictor had right and did not override with", "branches",     \
          1 },                                                                                                   \
        { "runahead_records_discarded", "Fetch-stage predictions thrown away by a re-steer", "branches", 1 },     \
    {                                                                                                            \
        "execute_repairs", "Mis-speculation repairs made at execute rather than at retire", "squashes", 1        \
    }

} // namespace Vanadis
} // namespace SST

#endif
