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

#ifndef _H_VANADIS_ROCC_INTERFACE
#define _H_VANADIS_ROCC_INTERFACE

#include <sst/core/output.h>
#include <sst/core/subcomponent.h>

#include <cinttypes>
#include <cstdint>
#include <vector>
#include <queue>

namespace SST {
namespace Vanadis {

class RoCCInstruction
{
public:
    RoCCInstruction(uint32_t func_code7, uint8_t rd, const bool xs1, const bool xs2, const bool xd) {
        this->func7 = func_code7;
        this->rd = rd;
        this->xs1 = xs1;
        this->xs2 = xs2;
        this->xd = xd;
    }
    ~RoCCInstruction() {}

    uint8_t func7;
    uint8_t rd;
    bool xs1;
    bool xs2;
    bool xd;
};

class RoCCCommand
{
public:
    RoCCCommand(RoCCInstruction* inst, uint64_t rs1, uint64_t rs2) {
        this->inst = inst;
        this->rs1 = rs1;
        this->rs2 = rs2;
    }
    ~RoCCCommand() {}
    RoCCInstruction* inst;
    uint64_t rs1;
    uint64_t rs2;
};

class RoCCResponse
{
public:
    RoCCResponse(uint8_t rd, uint64_t rd_val) {
        this->rd = rd;
        this->rd_val = rd_val;
    }
    ~RoCCResponse() {}
    uint8_t rd;
    uint64_t rd_val;
};

class VanadisRoCCInterface : public SST::SubComponent {
public:
    SST_ELI_REGISTER_SUBCOMPONENT_API(SST::Vanadis::VanadisRoCCInterface)

    SST_ELI_DOCUMENT_PARAMS({ "verbose", "Set the verbosity of output for the RoCC Accelerator Interface", "0" }, )

    SST_ELI_DOCUMENT_STATISTICS({ "roccs_issued", "Count number of rocc instructions that are issued", "operations", 1 })

    VanadisRoCCInterface(ComponentId_t id, Params& params) : SubComponent(id) {
        output = new SST::Output("[RoCC @t]: ", 2, 0xFFFFFFFF, SST::Output::STDOUT);

        stat_rocc_issued = registerStatistic<uint64_t>("roccs_issued", "1");
    }

    virtual ~VanadisRoCCInterface() {delete output;}

    virtual bool RoCCFull() = 0;
    virtual bool isBusy() = 0;
    virtual size_t roccQueueSize() = 0;
    virtual void push(RoCCCommand* rocc_me) = 0;
    virtual RoCCResponse* respond() = 0;
    virtual void tick(uint64_t cycle) = 0;

    virtual void init(unsigned int phase) = 0;

    SST::Output* output;

    Statistic<uint64_t>* stat_rocc_issued;

    // HOST INSTRUCTIONS RETIRED, published to the accelerator.
    //
    // An accelerator that wants to say how much work the core did BETWEEN two
    // of its own instructions cannot get that from anywhere else: it is handed
    // commands and cycles, and the retire count belongs to the core. The core
    // adds this cycle's retirements here before it ticks each interface, so a
    // RoCC reading it at the moment it executes a command sees a running total
    // that includes the cycle it is in. An accelerator that does not care simply
    // never reads it, and nothing in the core's behaviour depends on it.
    uint64_t host_insns_retired = 0;

    // HOST INSTRUCTIONS ON THE PROGRESS AXIS, published the same way and in the same place.
    //
    // `host_insns_retired` counts micro-ops, and it counts the polling an accelerator's
    // results are waited for. Neither is a property of the program: a decoder may break one
    // instruction into several, and how long a wait loop spins depends on how fast this
    // machine is. These two counters are the program's own instructions, one per
    // instruction, split by whether the instruction lies inside the span of wait and retry
    // code the core was given (`nmfc_wait_start`/`nmfc_wait_stop`) or outside it. An
    // accelerator measuring a window of the program counts `host_insns_progress`, because
    // its value at a given point is the same in this machine and in a functional simulator
    // of the same program. With no span given, every program instruction is progress.
    uint64_t host_insns_progress = 0;
    uint64_t host_insns_wait     = 0;
};

} // namespace Vanadis
} // namespace SST

#endif
