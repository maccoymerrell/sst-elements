// refdrive_hp -- drives the UNMODIFIED reference hashed perceptron from a
// recorded branch trace, the same way refdrive drives the reference TAGE-SC-L,
// and writes down what it predicted and what state it ended in.
//
// The reference keeps its weights and its shift registers private and offers no
// accessor for them, so this file is compiled with access checking off
// (-fno-access-control). Nothing is written through them; they are read once,
// at the end, to compare the port against them byte for byte.
//
// usage: refdrive_hp <trace> <out-prefix> [record-limit]

#include "branchtrace.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "hashed_perceptron.cc"

namespace
{

std::vector<uint8_t> state_bytes;

template <typename T>
void put(const T* p, size_t n)
{
  const uint8_t* b = reinterpret_cast<const uint8_t*>(p);
  state_bytes.insert(state_bytes.end(), b, b + n);
}

// The same fields, in the same order, as the port's serialization.
void serializeReference(hashed_perceptron& ref)
{
  state_bytes.clear();

  for (const auto& table : ref.tables) {
    for (const auto& w : table) {
      const int8_t v = static_cast<int8_t>(w.value());
      put(&v, sizeof(v));
    }
  }
  put(&ref.theta, sizeof(ref.theta));
  put(&ref.tc, sizeof(ref.tc));

  // Each shift register, as the four 64-bit words the port keeps, with the
  // words this length does not use left at zero in both.
  for (const auto& h : ref.ghist_words) {
    uint64_t words[4] = {0, 0, 0, 0};
    for (size_t i = 0; (i < h.words.size()) && (i < 4); ++i) {
      words[i] = h.words[i];
    }
    put(words, sizeof(words));
  }

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
    fprintf(stderr, "usage: refdrive_hp <trace> <out-prefix> [record-limit]\n");
    return 2;
  }

  const size_t limit = (argc > 3) ? (size_t)strtoull(argv[3], nullptr, 0) : 0;
  const std::vector<BranchRecord> trace = readBranchTrace(argv[1], limit);

  hashed_perceptron ref{champsim::modules::ModuleBuilder{}};
  ref.initialize_branch_predictor();

  std::vector<uint8_t> bits;
  bits.reserve(trace.size());

  for (size_t i = 0; i < trace.size(); ++i) {
    const BranchRecord& r = trace[i];

    const bool predicted = ref.predict_branch(champsim::address(r.pc), champsim::address(r.target), false,
                                              champsimBranchType(r.cls));
    if (0 == r.cls) { bits.push_back(predicted ? 1 : 0); }

    ref.last_branch_result(champsim::address(r.pc), champsim::address(r.target), r.taken != 0,
                           champsimBranchType(r.cls));
  }

  serializeReference(ref);

  const std::string prefix(argv[2]);
  writeFile(prefix + ".bits", bits.data(), bits.size());
  writeFile(prefix + ".state", state_bytes.data(), state_bytes.size());

  printf("refdrive_hp: %zu records, %zu conditional, state %zu bytes\n", trace.size(), bits.size(),
         state_bytes.size());
  return 0;
}
