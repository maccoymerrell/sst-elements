// A stand-in for ChampSim's modules.h, for the hashed perceptron reference.
// The real one drags in the whole simulator; the reference module itself needs
// only an address with a low-bits slice and a base class to derive from. The
// bit-width and counter helpers it uses are ChampSim's own headers, unchanged.
#ifndef VANADIS_TEST_CHAMPSIM_SHIM_HP_MODULES_H
#define VANADIS_TEST_CHAMPSIM_SHIM_HP_MODULES_H

#include "msl/bits.h"
#include "util/bit_enum.h"

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

  template <champsim::data::bits N>
  address slice_lower() const
  {
    return address(v_ & champsim::msl::bitmask(N));
  }

private:
  uint64_t v_;
};

namespace modules
{

struct ModuleBuilder {
};

struct branch_predictor {
  branch_predictor() = default;
  branch_predictor(ModuleBuilder) {}
  virtual ~branch_predictor() = default;

  template <typename T>
  struct register_module {
    register_module(const char*) {}
  };

  virtual void initialize_branch_predictor() {}
  virtual bool predict_branch(address, address, bool, uint8_t) { return false; }
  virtual void last_branch_result(address, address, bool, uint8_t) {}
};

} // namespace modules
} // namespace champsim

#endif
