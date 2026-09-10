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

#ifndef _H_VANADIS_ADD
#define _H_VANADIS_ADD

#include "inst/vinst.h"

#include <type_traits>

namespace SST {
namespace Vanadis {


template <typename gpr_format>
class VanadisAddInstruction : public virtual VanadisInstruction
{
    public:
        VanadisAddInstruction(const uint64_t addr, const uint32_t hw_thr,
            const VanadisDecoderOptions* isa_opts, const uint16_t dest,
            const uint16_t src_1, const uint16_t src_2) :
            VanadisInstruction(addr, hw_thr, isa_opts, 2, 1, 2, 1, 0, 0, 0, 0)
        {

            isa_int_regs_in[0]  = src_1;
            isa_int_regs_in[1]  = src_2;
            isa_int_regs_out[0] = dest;
        }

        VanadisAddInstruction*    clone() override { return new VanadisAddInstruction(*this); }
        VanadisFunctionalUnitType getInstFuncType() const override { return INST_INT_ARITH; }

        const char* getInstCode() const override
        {
                if(sizeof(gpr_format) == 8) {
                    return "ADD64";
                } else {
                    return "ADD32";
                }
        }

        void printToBuffer(char* buffer, size_t buffer_size) override
        {
            snprintf(
                buffer, buffer_size,
                "%s    %5" PRIu16 " <- %5" PRIu16 " + %5" PRIu16 " (phys: %5" PRIu16 " <- %5" PRIu16 " + %5" PRIu16 ")",
                getInstCode(), isa_int_regs_out[0], isa_int_regs_in[0], isa_int_regs_in[1], phys_int_regs_out[0],
                phys_int_regs_in[0], phys_int_regs_in[1]);
        }




        void instOp(VanadisRegisterFile* regFile,
        uint16_t phys_int_regs_out_0, uint16_t phys_int_regs_in_0,
        uint16_t phys_int_regs_in_1) override
        {
            using wrap_format = typename std::make_unsigned<gpr_format>::type;

        // OVERFLOW IS IGNORED, AND IGNORING IT HAS TO BE WELL DEFINED.
        // `[Unpriv. Ch. 4: the W forms "ignore overflow", and the 32-bit result
        // is sign-extended to 64 bits.]` The operands of a W form are read as
        // int32_t, so a sum that carries out of bit 31 overflows a signed C
        // integer, which the C++ standard does not define. The arithmetic is
        // therefore done in the unsigned type of the same width, where it wraps
        // by definition, and converted back: identical results on two's
        // complement, and no longer at the compiler's discretion.
            const gpr_format src_1 = regFile->getIntReg<gpr_format>(phys_int_regs_in_0);
            const gpr_format src_2 = regFile->getIntReg<gpr_format>(phys_int_regs_in_1);
            const gpr_format result = static_cast<gpr_format>(
                static_cast<wrap_format>(src_1) + static_cast<wrap_format>(src_2));
            regFile->setIntReg<gpr_format>(phys_int_regs_out_0, result);
        }


};




} // namespace Vanadis
} // namespace SST

#endif
