// A stand-in for ChampSim's instruction.h. The reference branch predictors read
// exactly one thing from it: the branch-type constants, whose values matter
// because the predictor switches on them.
#ifndef VANADIS_TEST_CHAMPSIM_SHIM_INSTRUCTION_H
#define VANADIS_TEST_CHAMPSIM_SHIM_INSTRUCTION_H

enum branch_type {
  BRANCH_DIRECT_JUMP = 0,
  BRANCH_INDIRECT,
  BRANCH_CONDITIONAL,
  BRANCH_DIRECT_CALL,
  BRANCH_INDIRECT_CALL,
  BRANCH_RETURN,
  BRANCH_OTHER,
  NOT_BRANCH
};

#endif
