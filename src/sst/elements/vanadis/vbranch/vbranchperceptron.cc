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

#include "vbranch/vbranchperceptron.h"
#include "vbranch/vbranchspecunit.h"

namespace SST {
namespace Vanadis {

// The hashed perceptron as a branch unit: sixteen tables of 4096 eight-bit
// weights, 64 KiB in all, with the same speculative history, the same return
// address stack and the same target buffer as the TAGE-SC-L unit, so that the
// two can be compared on equal terms.
class VanadisPerceptronBranchUnit : public VanadisSpeculativeBranchUnit<VanadisPerceptronCore>
{
public:
    SST_ELI_REGISTER_SUBCOMPONENT(
        VanadisPerceptronBranchUnit, "vanadis", "VanadisPerceptronBranchUnit", SST_ELI_ELEMENT_VERSION(1, 0, 0),
        "A hashed perceptron direction predictor with geometric history lengths, a speculative "
        "history that is repaired on a misprediction, a return address stack and a branch target buffer",
        SST::Vanadis::VanadisBranchUnit)

    SST_ELI_DOCUMENT_PARAMS(
        { "branch_entries", "Entries in the branch target buffer, which answers indirect jumps", "1536" },
        { "checkpoint_entries",
          "Entries in the checkpoint ring, replaced by the core's reorder-buffer depth at construction", "512" },
        { "verbose", "Verbosity of the unit's own output", "0" })

    SST_ELI_DOCUMENT_STATISTICS(VANADIS_BRANCH_SPECULATIVE_STATISTICS);

    VanadisPerceptronBranchUnit(ComponentId_t id, Params& params) :
        VanadisSpeculativeBranchUnit<VanadisPerceptronCore>(id, params)
    {}

    virtual ~VanadisPerceptronBranchUnit() {}
};

} // namespace Vanadis
} // namespace SST
