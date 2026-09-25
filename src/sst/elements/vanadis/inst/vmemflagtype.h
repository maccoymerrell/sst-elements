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

#ifndef _H_VANADIS_MEM_FLAG_TYPE
#define _H_VANADIS_MEM_FLAG_TYPE

namespace SST {
namespace Vanadis {

enum VanadisMemoryTransaction {
    MEM_TRANSACTION_NONE,
    MEM_TRANSACTION_LLSC_LOAD,
    MEM_TRANSACTION_LLSC_STORE,
    MEM_TRANSACTION_LOCK,
    // A cache-block clean (RISC-V Zicbom cbo.clean): no bytes read or written;
    // the line is written back below the coherence point and kept.
    MEM_TRANSACTION_CLEAN
};

inline const char*
getTransactionTypeString(VanadisMemoryTransaction transT)
{
    switch ( transT ) {
    case MEM_TRANSACTION_NONE:
        return "STD";
    case MEM_TRANSACTION_LLSC_LOAD:
        return "LLSC_LOAD";
    case MEM_TRANSACTION_LLSC_STORE:
        return "LLSC_STORE";
    case MEM_TRANSACTION_LOCK:
        return "LOCK";
    case MEM_TRANSACTION_CLEAN:
        return "CLEAN";
    default:
        return "UNKNOWN";
    }
};

} // namespace Vanadis
} // namespace SST

#endif
