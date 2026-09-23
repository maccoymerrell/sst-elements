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

#ifndef _H_VANADIS_ROCC_INST
#define _H_VANADIS_ROCC_INST

#include "inst/vinst.h"
#include "inst/vrocc.h"

#include "rocc/vroccinterface.h"

#include <cstdio>

namespace SST {
namespace Vanadis {

class VanadisRoCCInstruction : public VanadisInstruction
{

public:
    VanadisRoCCInstruction(
        const uint64_t addr, const uint32_t hw_thr, const VanadisDecoderOptions* isa_opts,
        const uint16_t rs1, const int16_t rs2, const uint16_t rd, const bool xd, const bool xs1,
        const bool xs2, uint32_t func_code7, uint8_t accelerator_id) :
        VanadisInstruction(addr, hw_thr, isa_opts, 2, 1, 2, 1, 0, 0, 0, 0)
    {

        // THE OPERAND FLAGS ARE HONOURED HERE, and they were not before.
        //
        // funct3 of a RoCC instruction is not a selector: its three bits say
        // which of rd, rs1 and rs2 the instruction actually uses. This
        // constructor used to declare all three unconditionally, so an
        // instruction that reads neither source -- a queue-depth probe, say --
        // was renamed against whatever producers happened to be writing the two
        // register numbers sitting in its unused fields, and waited for them. On
        // an out-of-order core that is a false dependence: it delays issue, it
        // is invisible in any answer, and it makes the same instruction cost
        // different amounts depending on code around it that it does not read.
        //
        // The unused slots are pointed at the ISA's ignore-writes register --
        // x0 on RISC-V -- rather than being removed, because the issue path
        // reads input slots 0 and 1 by index and a shorter list would put it out
        // of bounds. x0 has no producer, so naming it declares no dependence,
        // and a write to it is dropped by the register file.
        const uint16_t none = isa_opts->getRegisterIgnoreWrites();
        isa_int_regs_in[0] = xs1 ? rs1 : none;
        isa_int_regs_in[1] = xs2 ? static_cast<uint16_t>(rs2) : none;
        isa_int_regs_out[0] = xd ? rd : none;

        this->func7 = func_code7;
        // The same flag on the answer side. The coprocessor always produces a
        // response, because the core pops one instruction per response and an
        // instruction that produced none would wedge the queue -- so where xd
        // is clear the response must name the register that drops writes, or it
        // would write a physical register this instruction never renamed.
        this->rd = xd ? rd : static_cast<uint8_t>(none);
        this->xs1 = xs1;
        this->xs2 = xs2;
        this->xd = xd;

        switch (accelerator_id) {
            case 0:
                this->funcType = INST_ROCC0;
                this->instCode = "RoCC0";
                break;

            case 1:
                this->funcType = INST_ROCC1;
                this->instCode = "RoCC1";
                break;

            case 2:
                this->funcType = INST_ROCC2;
                this->instCode = "RoCC2";
                break;

            case 3:
                this->funcType = INST_ROCC3;
                this->instCode = "RoCC3";
                break;

            default:
                break;
        }

    }

    VanadisRoCCInstruction* clone() { return new VanadisRoCCInstruction(*this); }

    virtual VanadisFunctionalUnitType getInstFuncType() const { return funcType; }

    virtual const char* getInstCode() const { return instCode; }

    virtual void printToBuffer(char* buffer, size_t buffer_size)
    {
        snprintf(buffer, buffer_size, "%s", instCode);
    }

    virtual void execute(SST::Output* output, VanadisRegisterFile* regFile) {
        markExecuted();
    }

    const char* instCode;
    VanadisFunctionalUnitType funcType;
    uint8_t func7;
    uint8_t rd;
    bool xs1;
    bool xs2;
    bool xd;
};

} // namespace Vanadis
} // namespace SST

#endif
