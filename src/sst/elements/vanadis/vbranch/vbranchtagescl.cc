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

#include <sst_config.h>

#include "vbranch/vbranchspecunit.h"
#include "vbranch/vbranchtagescl.h"

namespace SST {
namespace Vanadis {

// TAGE-SC-L as a branch unit: a tagged predictor with geometric history
// lengths, a statistical corrector and a loop predictor, about 64 KiB of
// tables, with a 32-entry return address stack and the existing target buffer
// beside it for indirect jumps.
class VanadisTageSCLBranchUnit : public VanadisSpeculativeBranchUnit<VanadisTageSclCore>
{
public:
    SST_ELI_REGISTER_SUBCOMPONENT(
        VanadisTageSCLBranchUnit, "vanadis", "VanadisTageSCLBranchUnit", SST_ELI_ELEMENT_VERSION(1, 0, 0),
        "A TAGE-SC-L direction predictor with a speculative history that is repaired on a "
        "misprediction, a return address stack and a branch target buffer",
        SST::Vanadis::VanadisBranchUnit)

    SST_ELI_DOCUMENT_PARAMS(
        { "branch_entries", "Entries in the branch target buffer, which answers indirect jumps", "1536" },
        { "checkpoint_entries",
          "Entries in the checkpoint ring, replaced by the core's reorder-buffer depth at construction", "512" },
        { "verbose", "Verbosity of the unit's own output", "0" })

    SST_ELI_DOCUMENT_STATISTICS(VANADIS_BRANCH_SPECULATIVE_STATISTICS);

    VanadisTageSCLBranchUnit(ComponentId_t id, Params& params) :
        VanadisSpeculativeBranchUnit<VanadisTageSclCore>(id, params)
    {}

    virtual ~VanadisTageSCLBranchUnit() {}
};

} // namespace Vanadis
} // namespace SST
