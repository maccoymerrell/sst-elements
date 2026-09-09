// A stand-in for ChampSim's modules.h, holding only what the branch predictor
// modules under test actually use: an address wrapper and a base class for a
// branch predictor module. It exists so the unmodified reference sources can
// be compiled and driven from a recorded branch trace, with no simulator and
// no ChampSim build.
#ifndef VANADIS_TEST_CHAMPSIM_SHIM_MODULES_H
#define VANADIS_TEST_CHAMPSIM_SHIM_MODULES_H

#include <cstdint>

namespace champsim
{

class address
{
public:
  address() : v_(0) {}
  address(uint64_t v) : v_(v) {}

  template <typename T>
  T to() const
  {
    return static_cast<T>(v_);
  }

private:
  uint64_t v_;
};

namespace modules
{
struct branch_predictor {
  branch_predictor() = default;
};
} // namespace modules

} // namespace champsim

#endif
