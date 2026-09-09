// refdrive -- drives the UNMODIFIED reference TAGE-SC-L from a recorded branch
// trace, and writes down what it predicted and what state it ended in.
//
// The reference is a ChampSim branch-predictor module: it is told about one
// branch at a time and is given that branch's outcome immediately, so it never
// has a second prediction outstanding and never sees a wrong path. Driving it
// from the architected, in-order branch stream the core recorded reproduces
// exactly those conditions, which makes its output the thing the port has to
// match.
//
// Two files come out:
//   <out>.bits    one byte per conditional record: the direction predicted
//   <out>.state   the predictor's tables and history, in the same field order
//                 the port serializes them in
//
// usage: refdrive <trace> <out-prefix> [record-limit]

#include "branchtrace.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// The reference, compiled as it stands. Its tables and histories are file-scope
// variables in it, which is the first thing the port had to change and the
// reason it can be read directly here.
#include "tage_sc_l.cc"

namespace
{

std::vector<uint8_t> state_bytes;

template <typename T>
void put(const T* p, size_t n)
{
  const uint8_t* b = reinterpret_cast<const uint8_t*>(p);
  state_bytes.insert(state_bytes.end(), b, b + n);
}

// The same fields, in the same order, as the port's serialization. Anything
// that is only a per-prediction value -- the threshold, the hitting bank, the
// statistical corrector's sum -- is not state and is not here.
void serializeReference()
{
  state_bytes.clear();

  put(Bias, sizeof(Bias));
  put(BiasSK, sizeof(BiasSK));
  put(BiasBank, sizeof(BiasBank));
  put(GGEHLA, sizeof(GGEHLA));
  put(PGEHLA, sizeof(PGEHLA));
  put(LGEHLA, sizeof(LGEHLA));
  put(SGEHLA, sizeof(SGEHLA));
  put(TGEHLA, sizeof(TGEHLA));
  put(IGEHLA, sizeof(IGEHLA));
  put(IMGEHLA, sizeof(IMGEHLA));
  put(WG, sizeof(WG));
  put(WL, sizeof(WL));
  put(WS, sizeof(WS));
  put(WT, sizeof(WT));
  put(WP, sizeof(WP));
  put(WI, sizeof(WI));
  put(WIM, sizeof(WIM));
  put(WB, sizeof(WB));
  put(&updatethreshold, sizeof(updatethreshold));
  put(Pupdatethreshold, sizeof(Pupdatethreshold));
  put(&FirstH, sizeof(FirstH));
  put(&SecondH, sizeof(SecondH));
  put(use_alt_on_na, sizeof(use_alt_on_na));
  put(&TICK, sizeof(TICK));
  put(&Seed, sizeof(Seed));
  put(&WITHLOOP, sizeof(WITHLOOP));

  for (int i = 0; i < (1 << LOGB); i++) {
    put(&btable[i].pred, sizeof(int8_t));
    put(&btable[i].hyst, sizeof(int8_t));
  }
  for (int bank = 0; bank < 2; bank++) {
    const int     which = (bank == 0) ? 1 : BORN;
    const gentry* g = gtable[which];
    for (int i = 0; i < SizeTable[which]; i++) {
      put(&g[i].ctr, sizeof(int8_t));
      put(&g[i].tag, sizeof(uint32_t));
      put(&g[i].u, sizeof(int8_t));
    }
  }
  for (int i = 0; i < (1 << LOGL); i++) {
    put(&ltable[i].NbIter, sizeof(uint16_t));
    put(&ltable[i].confid, sizeof(uint8_t));
    put(&ltable[i].CurrentIter, sizeof(uint16_t));
    put(&ltable[i].TAG, sizeof(uint16_t));
    put(&ltable[i].age, sizeof(uint8_t));
    put(&ltable[i].dir, sizeof(bool));
  }

  // ... then the history, in the port's order.
  put(&GHIST, sizeof(GHIST));
  put(&phist, sizeof(phist));
  put(&ptghist, sizeof(ptghist));
  put(&IMLIcount, sizeof(IMLIcount));
  for (int i = 0; i <= NHIST; i++) {
    put(&ch_i[i].comp, sizeof(unsigned));
    put(&ch_t[0][i].comp, sizeof(unsigned));
    put(&ch_t[1][i].comp, sizeof(unsigned));
  }
  put(L_shist, sizeof(L_shist));
  put(S_slhist, sizeof(S_slhist));
  put(T_slhist, sizeof(T_slhist));
  put(IMHIST, sizeof(IMHIST));

  // The return address stack the port adds has no counterpart here, and is the
  // tail of the port's image; the comparison stops at this point.
}

void writeFile(const std::string& path, const void* data, size_t n)
{
  FILE* f = fopen(path.c_str(), "wb");
  if (nullptr == f) {
    fprintf(stderr, "cannot write %s\n", path.c_str());
    exit(2);
  }
  if (n > 0) { fwrite(data, 1, n, f); }
  fclose(f);
}

} // namespace

int main(int argc, char** argv)
{
  if (argc < 3) {
    fprintf(stderr, "usage: refdrive <trace> <out-prefix> [record-limit]\n");
    return 2;
  }

  const size_t limit = (argc > 3) ? (size_t)strtoull(argv[3], nullptr, 0) : 0;
  const std::vector<BranchRecord> trace = readBranchTrace(argv[1], limit);

  tage_sc_l ref{};
  ref.initialize_branch_predictor();

  std::vector<uint8_t> bits;
  bits.reserve(trace.size());

  struct Dbg {
    int32_t lsum, thres, bi;
    int16_t hitbank, altbank;
    int8_t  bim;
    uint8_t tage_pred, pred_taken, predloop, lvalid, highconf, medconf, lowconf, altconf;
  };
  std::vector<Dbg> dbg;

  // Intermediate images, so that a divergence is localised rather than only
  // reported at the end.
  std::vector<uint8_t> checkpoints;
  const size_t         CHECK_EVERY = 10000;

  for (size_t i = 0; i < trace.size(); ++i) {
    const BranchRecord& r = trace[i];

    const bool predicted = ref.predict_branch(champsim::address(r.pc));
    if (0 == r.cls) {
      bits.push_back(predicted ? 1 : 0);
      Dbg d; memset(&d, 0, sizeof(d));
      d.lsum = LSUM; d.thres = predictor.THRES; d.bi = BI;
      d.hitbank = HitBank; d.altbank = AltBank; d.bim = BIM;
      d.tage_pred = tage_pred; d.pred_taken = pred_taken; d.predloop = predloop; d.lvalid = LVALID;
      d.highconf = HighConf; d.medconf = MedConf; d.lowconf = LowConf; d.altconf = AltConf;
      dbg.push_back(d);
    }

    ref.last_branch_result(champsim::address(r.pc), champsim::address(r.target), r.taken != 0,
                           champsimBranchType(r.cls));

    if (((i + 1) % CHECK_EVERY) == 0) {
      serializeReference();
      // A 64-bit fingerprint of the whole image is enough to localise, and
      // keeps the file small.
      uint64_t h = 1469598103934665603ull;
      for (uint8_t b : state_bytes) {
        h = (h ^ b) * 1099511628211ull;
      }
      const uint8_t* hb = reinterpret_cast<const uint8_t*>(&h);
      checkpoints.insert(checkpoints.end(), hb, hb + sizeof(h));
    }
  }

  serializeReference();

  const std::string prefix(argv[2]);
  writeFile(prefix + ".bits", bits.data(), bits.size());
  writeFile(prefix + ".state", state_bytes.data(), state_bytes.size());
  writeFile(prefix + ".marks", checkpoints.data(), checkpoints.size());
  writeFile(prefix + ".dbg", dbg.data(), dbg.size() * sizeof(Dbg));

  printf("refdrive: %zu records, %zu conditional, state %zu bytes, %zu marks\n", trace.size(), bits.size(),
         state_bytes.size(), checkpoints.size() / 8);
  return 0;
}
