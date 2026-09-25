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

#ifndef _H_VANADIS_CACHE_CLEAN
#define _H_VANADIS_CACHE_CLEAN

#include "inst/vstore.h"

namespace SST {
namespace Vanadis {

// CBO.CLEAN (RISC-V Zicbom): write the cache block that holds the address in
// rs1 back to the point of coherence, and keep the copy.
//
// It is carried through the core as a store with no data: it takes a store
// queue slot when it is renamed, leaves the queue from the head of the reorder
// buffer on a store port like any store, and is sent to memory as a flush that
// does not invalidate (StandardMem::FlushAddr, inv = false). What it is not is
// an access: it writes no bytes, so the load/store queue never forwards from it,
// never holds a load behind it, and never charges a younger load a violation
// against it. The block is the cache line, 64 bytes, so the address is the line
// holding rs1; Zicbom takes no offset.
class VanadisCacheCleanInstruction : public VanadisStoreInstruction
{
public:
    static constexpr uint16_t BLOCK_BYTES = 64;

    VanadisCacheCleanInstruction(
        const uint64_t addr, const uint32_t hw_thr, const VanadisDecoderOptions* isa_opts, const uint16_t addrReg,
        const uint16_t zeroReg) :
        VanadisInstruction(addr, hw_thr, isa_opts, 2, 0, 2, 0, 0, 0, 0, 0),
        VanadisStoreInstruction(
            addr, hw_thr, isa_opts, addrReg, 0, zeroReg, BLOCK_BYTES, MEM_TRANSACTION_CLEAN, STORE_INT_REGISTER)
    {}

    VanadisCacheCleanInstruction* clone() override { return new VanadisCacheCleanInstruction(*this); }

    const char* getInstCode() const override { return "CBO_CLEAN"; }

    void printToBuffer(char* buffer, size_t buffer_size) override
    {
        snprintf(
            buffer, buffer_size, "CBO.CLEAN  block of memory[%5" PRIu16 "] (phys: block of memory[%5" PRIu16 "])",
            isa_int_regs_in[0], phys_int_regs_in[0]);
    }

    void computeStoreAddress(SST::Output* output, VanadisRegisterFile* reg, uint64_t* store_addr, uint16_t* op_width)
        override
    {
        const uint64_t a = reg->getIntReg<uint64_t>(phys_int_regs_in[0]);
        (*store_addr)    = a & ~uint64_t(BLOCK_BYTES - 1);
        (*op_width)      = BLOCK_BYTES;
    }
};

} // namespace Vanadis
} // namespace SST

#endif
