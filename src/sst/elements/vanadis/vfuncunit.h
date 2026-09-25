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

#ifndef _H_VANADIS_FUNCTIONAL_UNIT
#define _H_VANADIS_FUNCTIONAL_UNIT

#include <cinttypes>
#include <climits>
#include <cstdint>
#include <deque>
#include <unordered_set>

#include "inst/vinst.h"


namespace SST {
namespace Vanadis {

class VanadisFunctionalUnitInsRecord {
public:
    VanadisFunctionalUnitInsRecord(VanadisInstruction* base_ins, uint16_t cycles)
        : ins(base_ins), cycles_left(cycles) {}
    ~VanadisFunctionalUnitInsRecord() {}

    uint16_t getCycles() const { return cycles_left; }

    bool readyToExecute() const { return (0 == cycles_left); }

    void tick() { cycles_left = (cycles_left > 0) ? cycles_left - 1 : 0; }

    uint32_t getHardwareThread() const { return ins->getHWThread(); }

    VanadisInstruction* getInstruction() { return ins; }

private:
    VanadisInstruction* ins;
    uint16_t cycles_left;
};

// THE FLOATING-POINT COST CLASSES. A unit may be given a latency and an issue
// interval per class; a class it was not given (zero) costs the unit's own
// latency and one issue per cycle, which is what every unit did before the
// classes existed.
enum VanadisFPCostClass : int {
    VANADIS_FP_COST_ADD = 0,
    VANADIS_FP_COST_MUL,
    VANADIS_FP_COST_FMA,
    VANADIS_FP_COST_DIV_S,
    VANADIS_FP_COST_DIV_D,
    VANADIS_FP_COST_SQRT_S,
    VANADIS_FP_COST_SQRT_D,
    // The integer divider's two widths (divide and remainder alike): a
    // division unit may be given a latency and an issue interval for each.
    VANADIS_INT_COST_DIV_W,
    VANADIS_INT_COST_DIV_X,
    VANADIS_FP_COST_COUNT
};

class VanadisFunctionalUnit {

public:
    VanadisFunctionalUnit(uint16_t id, VanadisFunctionalUnitType unit_type, uint16_t lat)
        : fu_id(id), fu_type(unit_type), latency(lat), busy_cycles(0), held_cycles(0) {
        for (int i = 0; i < VANADIS_FP_COST_COUNT; i++) { class_latency[i] = 0; class_interval[i] = 0; }
    }

    ~VanadisFunctionalUnit() {
        for (auto q_itr = pending_execute.begin(); q_itr != pending_execute.end(); q_itr++) {
            delete (*q_itr);
        }
    }

    VanadisFunctionalUnitType getType() const { return fu_type; }

    bool isInstructionSlotFree() const { return busy_cycles == 0; }

    // Per-class latency and issue interval (zero: the unit's own latency, one
    // issue per cycle). An ITERATIVE unit -- a divider -- is expressed as an
    // interval above one: it accepts nothing else until the interval has
    // passed, which is what "blocks subsequent similar operations" means.
    void setClassCost(int cls, uint16_t lat, uint16_t interval) {
        if (cls >= 0 && cls < VANADIS_FP_COST_COUNT) {
            class_latency[cls]  = lat;
            class_interval[cls] = interval;
        }
    }

    void insertInstruction(VanadisInstruction* ins) {
        const int cls      = ins->getFPCostClass();
        uint16_t  lat      = latency;
        uint16_t  interval = 1;
        if (cls >= 0 && cls < VANADIS_FP_COST_COUNT) {
            if (class_latency[cls] > 0) { lat = class_latency[cls]; }
            if (class_interval[cls] > 0) { interval = class_interval[cls]; }
        }
        pending_execute.push_back(new VanadisFunctionalUnitInsRecord(ins, lat));
        busy_cycles = interval;
        held_cycles += interval;
    }

    uint64_t heldCycles() const { return held_cycles; }

    uint16_t getUnitID() const { return fu_id; }

    void tick(const uint64_t cycle, SST::Output* output, std::vector<VanadisRegisterFile*>& regFile) {
        int k_in=0;
        for(auto q_itr = pending_execute.begin(); q_itr != pending_execute.end();) {
            VanadisFunctionalUnitInsRecord* q_item = (*q_itr);

            if(q_item->readyToExecute())
            {
                VanadisInstruction* inner_ins = q_item->getInstruction();
                inner_ins->execute(output, regFile);

                if(LIKELY(inner_ins->completedExecution()))
                {
                    // Delete the record entry for functional unit if the instruction marked itself executed
                    delete q_item;

                    // ready to execute, remove from pending queue
                    q_itr = pending_execute.erase(q_itr);
                }
                else
                {
                    q_itr++;
                }
            }
            else
            {
                q_item->tick();
                q_itr++;
            }
        }

        if (busy_cycles > 0) { busy_cycles--; }
    }

    void clearByHWThreadID(SST::Output* output, const uint16_t hw_thr) {
        output->verbose(CALL_INFO, 16, 0, "-> Function Unit, clearing by hardware thread %" PRIu32 "...\n", hw_thr);

        for (auto q_itr = pending_execute.begin(); q_itr != pending_execute.end();) {
            // if we get a hardware thread match, remove and carry out
            if ((*q_itr)->getHardwareThread() == hw_thr) {
                delete (*q_itr);
                q_itr = pending_execute.erase(q_itr);
            } else {
                q_itr++;
            }
        }
    }

    // A SQUASH AT EXECUTE takes only the younger half of a thread's window, so
    // the unit is asked about instructions rather than about a thread.
    void clearInstructions(const std::unordered_set<VanadisInstruction*>& victims) {
        if (victims.empty()) { return; }

        for (auto q_itr = pending_execute.begin(); q_itr != pending_execute.end();) {
            if (victims.count((*q_itr)->getInstruction()) > 0) {
                delete (*q_itr);
                q_itr = pending_execute.erase(q_itr);
            } else {
                q_itr++;
            }
        }
    }

    void print(SST::Output* output) {
        uint16_t index = 0;

        for (auto q_itr = pending_execute.begin(); q_itr != pending_execute.end(); q_itr++) {
            VanadisFunctionalUnitInsRecord* q_front = pending_execute.front();
            output->verbose(CALL_INFO, 16, 0, "----> func-unit: %" PRIu16 " %" PRIu16 " entries / entry: %" PRIu16 " / %s / 0x%" PRI_ADDR " / cycles: %" PRIu16 " out of %" PRIu16 "\n",
                fu_id, (uint16_t) pending_execute.size(), index++, q_front->getInstruction()->getInstCode(),
                q_front->getInstruction()->getInstructionAddress(), q_front->getCycles(), latency);
        }
    }

private:
    std::deque<VanadisFunctionalUnitInsRecord*> pending_execute;

    const uint16_t latency;
    VanadisFunctionalUnitType fu_type;
    const uint16_t fu_id;
    uint16_t busy_cycles;   ///< cycles before the unit accepts another operation
    uint64_t held_cycles;   ///< sum of the issue intervals it has been held for
    uint16_t class_latency[VANADIS_FP_COST_COUNT];
    uint16_t class_interval[VANADIS_FP_COST_COUNT];
};

} // namespace Vanadis
} // namespace SST

#endif
