// The recorded branch trace: one 24-byte record per retired branch, in program
// order, as the core writes it when `branch_trace_file` is set.
#ifndef VANADIS_TEST_BRANCH_TRACE_H
#define VANADIS_TEST_BRANCH_TRACE_H

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

struct BranchRecord {
  uint64_t pc;
  uint64_t target; // the branch's own taken target when the decode knows it,
                   // otherwise the address it went to
  uint8_t  cls;    // 0 conditional, 1 direct jump, 2 direct call,
                   // 3 indirect jump, 4 indirect call, 5 return
  uint8_t  taken;
  uint8_t  pad[6];
};

inline std::vector<BranchRecord> readBranchTrace(const char* path, size_t limit)
{
  FILE* f = fopen(path, "rb");
  if (nullptr == f) {
    fprintf(stderr, "cannot open branch trace %s\n", path);
    exit(2);
  }

  std::vector<BranchRecord> out;
  BranchRecord              r;
  while (1 == fread(&r, sizeof(r), 1, f)) {
    out.push_back(r);
    if ((limit > 0) && (out.size() >= limit)) { break; }
  }
  fclose(f);

  if (out.empty()) {
    fprintf(stderr, "branch trace %s is empty\n", path);
    exit(2);
  }
  return out;
}

// The branch-type constant the ChampSim reference switches on, for a class the
// trace records.
inline uint8_t champsimBranchType(uint8_t cls)
{
  switch (cls) {
  case 0: return 2; // BRANCH_CONDITIONAL
  case 1: return 0; // BRANCH_DIRECT_JUMP
  case 2: return 3; // BRANCH_DIRECT_CALL
  case 3: return 1; // BRANCH_INDIRECT
  case 4: return 4; // BRANCH_INDIRECT_CALL
  case 5: return 5; // BRANCH_RETURN
  }
  return 6;         // BRANCH_OTHER
}

#endif
