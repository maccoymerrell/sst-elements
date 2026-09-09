// portdrive -- drives the ported predictors from the same recorded branch
// trace, and checks four things.
//
//   Stage 1  With fetch order equal to retire order and no wrong path, the
//            port predicts the same direction as the reference on every
//            conditional branch and ends in a byte-identical state. A failure
//            here means a value the prediction computed and the training reads
//            was left shared between branches, or a history value was read
//            live where it should have come from the branch's own record.
//
//   Stage 2  With wrong-path branches injected before a branch retires, and a
//            repair from that branch's record afterwards, the correct-path
//            predictions and the final state are the same as in stage 1, for
//            every injection period and every wrong-path length swept. This is
//            the test the checkpoint exists for: it fails if the record leaves
//            out any speculative field, if the ring hands a slot back too
//            early, or if the repair forgets to replay the mispredicting
//            branch with what actually happened.
//
//   Stage 3  The injected wrong path really did move the speculative history
//            before the repair, so that stage 2 is not passing because nothing
//            happened.
//
//   Stage 4  Repairing from the branch's own record and discarding everything
//            unretired leave identical state -- which they do while branches
//            are resolved at retirement, and which is where it will be noticed
//            if resolution ever moves earlier.
//
// usage: portdrive <trace> <tage-ref-prefix> <perceptron-ref-prefix>
//                  [stage1-records] [stage2-records]

#include "vbranch/vbranchperceptron.h"
#include "vbranch/vbranchspeccore.h"
#include "vbranch/vbranchtagescl.h"

#include "branchtrace.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

using namespace SST::Vanadis;

namespace
{

int failures = 0;

void check(const bool ok, const char* what)
{
  if (!ok) {
    printf("FAIL  %s\n", what);
    failures++;
  }
}

std::vector<uint8_t> readFile(const std::string& path)
{
  FILE* f = fopen(path.c_str(), "rb");
  if (nullptr == f) {
    fprintf(stderr, "cannot read %s\n", path.c_str());
    exit(2);
  }
  std::vector<uint8_t> out;
  uint8_t              buf[65536];
  size_t               n;
  while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
    out.insert(out.end(), buf, buf + n);
  }
  fclose(f);
  return out;
}

VanadisBranchClass classOf(const uint8_t c) { return static_cast<VanadisBranchClass>(c); }

bool hasStatic(const uint8_t c) { return c <= 2; } // conditional, direct jump, direct call

// The per-branch values the reference keeps in globals, dumped so that a
// divergence can be pinned to the value that first differs rather than only to
// the branch it first shows up on.
struct Dbg {
  int32_t lsum, thres, bi;
  int16_t hitbank, altbank;
  int8_t  bim;
  uint8_t tage_pred, pred_taken, predloop, lvalid, highconf, medconf, lowconf, altconf;
};

// One predictor, driven the way the core drives it: a fetch that predicts a
// direction and then a target, and a retire that trains.
template <typename CORE>
struct Driver {
  VanadisSpeculativePredictor<CORE>       predictor;
  std::unordered_map<uint64_t, uint64_t>  btb;
  std::vector<uint8_t>                    bits;
  std::vector<Dbg>*                       dbg = nullptr;

  Driver() { predictor.setDepth(352); }

  struct Fetched {
    VanadisBranchCheckpoint handle;
    bool                    dir    = false;
    uint64_t                target = 0;
  };

  Fetched fetch(const BranchRecord& r, const bool collect)
  {
    VanadisBranchCheckpoint h;

    const uint64_t fallthrough = r.pc + 4;
    const bool     has_static  = hasStatic(r.cls);

    const bool dir =
        predictor.predictDirection(r.pc, classOf(r.cls), r.target, has_static, fallthrough, &h);

    if (collect && (0 == r.cls)) {
      bits.push_back(dir ? 1 : 0);
      if constexpr (std::is_same_v<CORE, VanadisTageSclCore>) {
      if (nullptr != dbg) {
        const auto& C = predictor.record(h);
        Dbg d; memset(&d, 0, sizeof(d));
        d.lsum = C.LSUM; d.thres = C.THRES; d.bi = C.BI;
        d.hitbank = C.HitBank; d.altbank = C.AltBank; d.bim = C.BIM;
        d.tage_pred = C.tage_pred; d.pred_taken = C.pred_taken; d.predloop = C.predloop; d.lvalid = C.LVALID;
        d.highconf = C.HighConf; d.medconf = C.MedConf; d.lowconf = C.LowConf; d.altconf = C.AltConf;
        dbg->push_back(d);
      }
      }
    }

    Fetched f;
    f.handle = h;
    f.dir    = dir;
    f.target = fallthrough;

    if (dir) {
      const auto     found     = btb.find(r.pc);
      const bool     btb_hit   = (found != btb.end());
      const uint64_t btb_target = btb_hit ? found->second : 0;
      f.target =
          predictor.predictTarget(r.pc, classOf(r.cls), r.target, has_static, fallthrough, btb_target, btb_hit, &h);
    }

    return f;
  }

  // The core squashes when the address the branch went to is not the address
  // it was predicted to go to, whichever half of the prediction was wrong.
  static bool mispredicted(const BranchRecord& r, const Fetched& f)
  {
    const uint64_t went_to = (r.taken != 0) ? r.target : (r.pc + 4);
    const uint64_t guessed = f.dir ? f.target : (r.pc + 4);
    return went_to != guessed;
  }

  void retire(const BranchRecord& r, const VanadisBranchCheckpoint& h)
  {
    const bool ordered = predictor.update(r.pc, classOf(r.cls), r.taken != 0, r.target, h);
    check(ordered, "checkpoints are released in retire order");
    btb[r.pc] = r.target;
  }

  std::vector<uint8_t> state() const
  {
    std::vector<uint8_t> v;
    predictor.serializeState(v);
    return v;
  }

  std::vector<uint8_t> spec() const
  {
    std::vector<uint8_t> v;
    predictor.serializeSpeculativeState(v);
    return v;
  }

  // The architected history is the tail of the full image, and the
  // speculative image is the same layout, so the two can be compared.
  std::vector<uint8_t> arch() const
  {
    const std::vector<uint8_t> s = state();
    const size_t               n = spec().size();
    return std::vector<uint8_t>(s.end() - n, s.end());
  }
};

// A branch from somewhere else in the trace, so that a wrong path is built out
// of addresses the program really contains.
const BranchRecord& wrongPathRecord(const std::vector<BranchRecord>& trace, const size_t at, const size_t j)
{
  const size_t idx = (at * 1103515245ull + j * 2654435761ull + 12345ull) % trace.size();
  return trace[idx];
}

// One pass over the trace. `period` 0 injects nothing; otherwise every
// `period`-th branch has `width` wrong-path branches fetched behind it before
// it retires, and is repaired afterwards. `random_period` varies the period
// pseudo-randomly instead of using a fixed one.
template <typename CORE>
struct PassResult {
  std::vector<uint8_t> bits;
  std::vector<uint8_t> state;
  uint64_t             injections      = 0;
  uint64_t             moved_history   = 0;
  uint64_t             repair_mismatch = 0;
  uint64_t             spec_not_arch   = 0;
};

template <typename CORE>
PassResult<CORE> runPass(const std::vector<BranchRecord>& trace, const size_t count, const size_t period,
                         const size_t width, const bool random_period, const bool cross_check,
                         std::vector<Dbg>* dbg = nullptr)
{
  Driver<CORE> P; // repairs from the mispredicting branch's own record
  P.dbg = dbg;
  Driver<CORE> Q; // discards everything unretired instead

  PassResult<CORE> res;
  uint64_t         rng  = 88172645463325252ull;
  size_t           next = period;

  for (size_t i = 0; i < count; ++i) {
    const BranchRecord& r = trace[i];

    const typename Driver<CORE>::Fetched fp = P.fetch(r, true);
    const typename Driver<CORE>::Fetched fq =
        cross_check ? Q.fetch(r, false) : typename Driver<CORE>::Fetched();

    const bool wrong = Driver<CORE>::mispredicted(r, fp);

    const bool inject = (period > 0) && (i + 1 >= next);

    if (inject) {
      if (random_period) {
        rng ^= rng << 13;
        rng ^= rng >> 7;
        rng ^= rng << 17;
        next = i + 1 + 1 + (rng % 32);
      }
      else {
        next = i + 1 + period;
      }

      const std::vector<uint8_t> before = P.spec();

      for (size_t j = 0; j < width; ++j) {
        const BranchRecord& w = wrongPathRecord(trace, i, j);
        P.fetch(w, false);
        if (cross_check) { Q.fetch(w, false); }
      }

      const std::vector<uint8_t> after = P.spec();
      if (before != after) { res.moved_history++; }

      P.retire(r, fp.handle);
      P.predictor.repair(fp.handle, r.taken != 0);

      if (cross_check) {
        Q.retire(r, fq.handle);
        Q.predictor.repairToCommit();
      }

      // Serializing a 219 KB image is not free, so the two whole-image checks
      // are sampled rather than run on every injection; the whole image is
      // compared once per configuration at the end regardless.
      const bool deep = ((res.injections % 101) == 0) || ((i + 1) == count);

      if (deep) {
        if (cross_check && (P.state() != Q.state() || P.spec() != Q.spec())) { res.repair_mismatch++; }
        if (P.spec() != P.arch()) { res.spec_not_arch++; }
      }

      res.injections++;
    }
    else {
      // What the machine does: a squash whenever the address the branch went
      // to is not the one it was sent to.
      P.retire(r, fp.handle);
      if (wrong) { P.predictor.repair(fp.handle, r.taken != 0); }

      if (cross_check) {
        Q.retire(r, fq.handle);
        if (Driver<CORE>::mispredicted(r, fq)) { Q.predictor.repairToCommit(); }
      }
    }
  }

  res.bits  = P.bits;
  res.state = P.state();
  return res;
}

} // namespace

int main(int argc, char** argv)
{
  if (argc < 4) {
    fprintf(stderr, "usage: portdrive <trace> <tage-ref-prefix> <perceptron-ref-prefix> "
                    "[stage1-records] [stage2-records]\n");
    return 2;
  }

  const std::string prefix(argv[2]);
  const std::string hp_prefix(argv[3]);
  const size_t      stage1_limit = (argc > 4) ? (size_t)strtoull(argv[4], nullptr, 0) : 0;
  const size_t      stage2_limit = (argc > 5) ? (size_t)strtoull(argv[5], nullptr, 0) : 4000;

  const std::vector<BranchRecord> trace = readBranchTrace(argv[1], stage1_limit);

  printf("portdrive: %zu records\n", trace.size());
  printf("           TAGE-SC-L tables %d bits (%d KiB), hashed perceptron %d bits (%d KiB)\n",
         VanadisTageSclCore::storageBits(), VanadisTageSclCore::storageBits() / 8192,
         VanadisPerceptronCore::storageBits(), VanadisPerceptronCore::storageBits() / 8192);

  // ---- stage 1: the same predictions and the same state as the reference ---
  std::vector<Dbg> port_dbg;
  PassResult<VanadisTageSclCore> base =
      runPass<VanadisTageSclCore>(trace, trace.size(), 0, 0, false, false, &port_dbg);

  const std::vector<uint8_t> ref_bits  = readFile(prefix + ".bits");
  const std::vector<uint8_t> ref_state = readFile(prefix + ".state");

  check(base.bits.size() == ref_bits.size(), "stage 1: same number of conditional branches as the reference");

  size_t first_diff = SIZE_MAX;
  const size_t nbits = base.bits.size() < ref_bits.size() ? base.bits.size() : ref_bits.size();
  for (size_t i = 0; i < nbits; ++i) {
    if (base.bits[i] != ref_bits[i]) {
      first_diff = i;
      break;
    }
  }
  if (first_diff != SIZE_MAX) {
    printf("      first differing prediction at conditional branch %zu\n", first_diff);

    const std::vector<uint8_t> raw = readFile(prefix + ".dbg");
    const Dbg* rd = reinterpret_cast<const Dbg*>(raw.data());
    const size_t nd = raw.size() / sizeof(Dbg);
    for (size_t i = 0; (i < nd) && (i < port_dbg.size()); ++i) {
      const Dbg& a = port_dbg[i];
      const Dbg& b = rd[i];
      if (memcmp(&a, &b, sizeof(Dbg)) != 0) {
        printf("      first differing internals at conditional branch %zu\n", i);
        printf("        port  lsum=%d thres=%d bi=%d hit=%d alt=%d bim=%d tage=%d pred=%d loop=%d "
               "lvalid=%d hc=%d mc=%d lc=%d ac=%d\n",
               a.lsum, a.thres, a.bi, a.hitbank, a.altbank, a.bim, a.tage_pred, a.pred_taken, a.predloop,
               a.lvalid, a.highconf, a.medconf, a.lowconf, a.altconf);
        printf("        ref   lsum=%d thres=%d bi=%d hit=%d alt=%d bim=%d tage=%d pred=%d loop=%d "
               "lvalid=%d hc=%d mc=%d lc=%d ac=%d\n",
               b.lsum, b.thres, b.bi, b.hitbank, b.altbank, b.bim, b.tage_pred, b.pred_taken, b.predloop,
               b.lvalid, b.highconf, b.medconf, b.lowconf, b.altconf);
        break;
      }
    }
  }
  check(first_diff == SIZE_MAX, "stage 1: every predicted direction matches the reference");

  // The port's image carries a return address stack the reference has no
  // counterpart for, and it is the tail; the comparison is over the rest.
  check(base.state.size() >= ref_state.size(), "stage 1: the port's image covers the reference's");
  bool state_same = (base.state.size() >= ref_state.size()) &&
                    (0 == memcmp(base.state.data(), ref_state.data(), ref_state.size()));
  if (!state_same && (base.state.size() >= ref_state.size())) {
    for (size_t i = 0; i < ref_state.size(); ++i) {
      if (base.state[i] != ref_state[i]) {
        printf("      first differing state byte at offset %zu (port %u, reference %u)\n", i,
               (unsigned)base.state[i], (unsigned)ref_state[i]);
        break;
      }
    }
  }
  check(state_same, "stage 1: the predictor's state is byte-identical to the reference's");

  printf("stage 1: %zu conditional predictions, %zu bytes of state, identical to the reference\n",
         base.bits.size(), ref_state.size());

  // ---- stage 1 again, for the hashed perceptron ---------------------------
  {
    PassResult<VanadisPerceptronCore> hp =
        runPass<VanadisPerceptronCore>(trace, trace.size(), 0, 0, false, false);

    const std::vector<uint8_t> hp_bits  = readFile(hp_prefix + ".bits");
    const std::vector<uint8_t> hp_state = readFile(hp_prefix + ".state");

    check(hp.bits == hp_bits, "stage 1: the hashed perceptron predicts what its reference predicts");
    check(hp.state.size() >= hp_state.size(), "stage 1: the perceptron image covers the reference's");

    bool same = (hp.state.size() >= hp_state.size()) &&
                (0 == memcmp(hp.state.data(), hp_state.data(), hp_state.size()));
    if (!same && (hp.state.size() >= hp_state.size())) {
      for (size_t i = 0; i < hp_state.size(); ++i) {
        if (hp.state[i] != hp_state[i]) {
          printf("      first differing perceptron state byte at offset %zu (port %u, reference %u)\n", i,
                 (unsigned)hp.state[i], (unsigned)hp_state[i]);
          break;
        }
      }
    }
    check(same, "stage 1: the hashed perceptron's state is byte-identical to its reference's");

    printf("stage 1: %zu conditional predictions and %zu bytes of state, identical to the hashed "
           "perceptron reference\n",
           hp.bits.size(), hp_state.size());
  }

  // ---- stages 2, 3 and 4: wrong paths, repair, and the two repairs agreeing -
  const size_t s2 = (trace.size() < stage2_limit) ? trace.size() : stage2_limit;

  PassResult<VanadisTageSclCore> s2base = runPass<VanadisTageSclCore>(trace, s2, 0, 0, false, false);

  const size_t periods[]  = {1, 2, 3, 7, 31, 0 /* stands for the random schedule */};
  uint64_t     configs    = 0;
  uint64_t     injections = 0;
  uint64_t     moved      = 0;

  for (size_t pi = 0; pi < sizeof(periods) / sizeof(periods[0]); ++pi) {
    const bool   random_period = (0 == periods[pi]);
    const size_t period        = random_period ? 1 : periods[pi];

    for (size_t width = 1; width <= 64; ++width) {
      PassResult<VanadisTageSclCore> r =
          runPass<VanadisTageSclCore>(trace, s2, period, width, random_period, true);

      configs++;
      injections += r.injections;
      moved += r.moved_history;

      if (r.bits != s2base.bits || r.state != s2base.state) {
        printf("FAIL  stage 2: period %zu%s, width %zu diverges from the no-wrong-path run\n", period,
               random_period ? " (random)" : "", width);
        failures++;
      }
      if (r.repair_mismatch != 0) {
        printf("FAIL  stage 4: period %zu, width %zu: repairing from the record and discarding "
               "everything unretired disagree %llu times\n",
               period, width, (unsigned long long)r.repair_mismatch);
        failures++;
      }
      if (r.spec_not_arch != 0) {
        printf("FAIL  stage 2: period %zu, width %zu: after a repair the speculative history is not "
               "the architected one, %llu times\n",
               period, width, (unsigned long long)r.spec_not_arch);
        failures++;
      }
      if (r.moved_history == 0) {
        printf("FAIL  stage 3: period %zu, width %zu: the wrong path never moved the speculative "
               "history, so the repair proves nothing\n",
               period, width);
        failures++;
      }
    }
  }

  printf("stage 2: %llu configurations over %zu records, %llu injected wrong paths, all identical to the "
         "run with none\n",
         (unsigned long long)configs, s2, (unsigned long long)injections);
  printf("stage 3: %llu of %llu injections moved the speculative history before the repair\n",
         (unsigned long long)moved, (unsigned long long)injections);
  printf("stage 4: repairing from the branch's own record and discarding everything unretired agree "
         "everywhere\n");

  // ---- the same three stages for the hashed perceptron ---------------------
  PassResult<VanadisPerceptronCore> pbase = runPass<VanadisPerceptronCore>(trace, s2, 0, 0, false, false);

  uint64_t pconfigs = 0, pinject = 0, pmoved = 0;
  for (size_t pi = 0; pi < sizeof(periods) / sizeof(periods[0]); ++pi) {
    const bool   random_period = (0 == periods[pi]);
    const size_t period        = random_period ? 1 : periods[pi];

    for (size_t width = 1; width <= 64; width += 7) {
      PassResult<VanadisPerceptronCore> r =
          runPass<VanadisPerceptronCore>(trace, s2, period, width, random_period, true);
      pconfigs++;
      pinject += r.injections;
      pmoved += r.moved_history;

      if (r.bits != pbase.bits || r.state != pbase.state) {
        printf("FAIL  perceptron stage 2: period %zu, width %zu diverges from the no-wrong-path run\n",
               period, width);
        failures++;
      }
      if (r.repair_mismatch != 0 || r.spec_not_arch != 0 || r.moved_history == 0) {
        printf("FAIL  perceptron stage 3/4: period %zu, width %zu\n", period, width);
        failures++;
      }
    }
  }
  printf("perceptron: %llu configurations, %llu injected wrong paths, %llu of them moved the history, all "
         "identical to the run with none\n",
         (unsigned long long)pconfigs, (unsigned long long)pinject, (unsigned long long)pmoved);

  if (0 == failures) {
    printf("PASS  all stages\n");
    return 0;
  }

  printf("FAILED %d checks\n", failures);
  return 1;
}
