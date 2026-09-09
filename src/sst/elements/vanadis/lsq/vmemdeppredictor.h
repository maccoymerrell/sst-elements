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

#ifndef _H_VANADIS_MEM_DEP_PREDICTOR
#define _H_VANADIS_MEM_DEP_PREDICTOR

// MEMORY-DEPENDENCE PREDICTION.
//
// A load that issues past an older store whose address nobody has computed yet
// is guessing that the two do not touch the same bytes. When the guess is
// wrong the machine has to throw the load and everything younger away and
// fetch again, which costs a whole pipeline refill. A memory-dependence
// predictor is the structure that stops a load which has been wrong before from
// guessing again.
//
// Two are implemented here. Both are indexed by the load's instruction address
// and both are UNTAGGED -- two loads whose addresses collide in the index share
// one prediction, which is the accepted cost of a table with no tags and the
// reason the default is large.
//
//   COUNTER   a two-bit saturating counter per load address. A load that causes
//             a flush counts up; a load that retires without having caused one
//             counts down; above 1 the load waits for every older store.
//
//   STORE_PC  the address of the store instruction that last made this load
//             flush. The load waits only while that particular store is in the
//             queue ahead of it, which is narrower than making it wait for all
//             of them, at the cost of a 64-bit field instead of two bits.
//
// NONE never holds a load, and exists so that the maximum-speculation case can
// be measured and tested.

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace SST {
namespace Vanadis {

// What the queue lets the predictor ask about the stores ahead of a load,
// without handing it the queue. `has_unresolved` is true when at least one
// older store has no address yet -- the only situation in which the predictor
// is consulted at all -- and `containsStorePC` answers whether a named store
// instruction is one of the older ones.
class VanadisStoreQView
{
public:
    virtual ~VanadisStoreQView() {}
    virtual bool containsStorePC(uint64_t store_pc) const = 0;
};

class VanadisMemDepPredictor
{
public:
    virtual ~VanadisMemDepPredictor() {}

    /// May the load at `load_pc` be issued past older stores whose addresses
    /// are not known? False means hold it.
    virtual bool speculate(uint64_t load_pc, const VanadisStoreQView& older) = 0;

    /// The load at `load_pc` read bytes the store at `store_pc` then wrote.
    virtual void violated(uint64_t load_pc, uint64_t store_pc) = 0;

    /// The load at `load_pc` retired without having caused a flush.
    /// `speculated` says whether it actually went past an unknown store.
    virtual void retiredClean(uint64_t load_pc, bool speculated) = 0;

    /// Everything the table has learned about a thread's program is discarded.
    /// Called when the thread's address space is replaced, and at no other
    /// time: a table wiped on every branch mis-predict would stay empty.
    virtual void reset() = 0;

    virtual const char* name() const = 0;
};

// ---------------------------------------------------------------------------
// A. Two-bit saturating counter per load address. The default.
// ---------------------------------------------------------------------------
//
// The rule, stated plainly, because its behaviour surprises people: the
// threshold is "above 1" and the decrement fires on every clean retire, so a
// load that always aliases settles into a two-step cycle. Its counter reaches
// 2, it is held, it retires clean, the counter falls to 1, it speculates, it
// violates, the counter returns to 2. About half its executions are held and
// half violate. That is what the rule says and it is what this does.
// `decay_on_speculated_only` selects the other reading: a held load produces no
// evidence, so its counter does not fall and a load that always aliases is held
// for good after its second violation.
class VanadisMemDepCounter : public VanadisMemDepPredictor
{
public:
    VanadisMemDepCounter(size_t entries, bool decay_on_speculated_only) :
        mask_(entries - 1), decay_spec_only_(decay_on_speculated_only), table_(entries, 0)
    {}

    bool speculate(uint64_t load_pc, const VanadisStoreQView&) override { return table_[index(load_pc)] <= 1; }

    void violated(uint64_t load_pc, uint64_t) override
    {
        uint8_t& c = table_[index(load_pc)];
        if ( c < 3 ) { c++; }
    }

    void retiredClean(uint64_t load_pc, bool speculated) override
    {
        if ( decay_spec_only_ && !speculated ) { return; }
        uint8_t& c = table_[index(load_pc)];
        if ( c > 0 ) { c--; }
    }

    void reset() override { std::fill(table_.begin(), table_.end(), static_cast<uint8_t>(0)); }

    const char* name() const override { return "counter"; }

private:
    size_t index(uint64_t pc) const { return static_cast<size_t>((pc >> 2) & mask_); }

    const uint64_t       mask_;
    const bool           decay_spec_only_;
    std::vector<uint8_t> table_;
};

// ---------------------------------------------------------------------------
// B. Load address -> the address of the store that last made it flush.
// ---------------------------------------------------------------------------
//
// No decay is needed. The hold is already conditional on the named store being
// one of the ones ahead of this load, so an entry that has stopped describing
// the program costs nothing the moment that store stops appearing.
class VanadisMemDepStorePC : public VanadisMemDepPredictor
{
public:
    VanadisMemDepStorePC(size_t entries) : mask_(entries - 1), valid_(entries, false), store_pc_(entries, 0) {}

    bool speculate(uint64_t load_pc, const VanadisStoreQView& older) override
    {
        const size_t i = index(load_pc);
        if ( !valid_[i] ) { return true; }
        return !older.containsStorePC(store_pc_[i]);
    }

    void violated(uint64_t load_pc, uint64_t store_pc) override
    {
        const size_t i = index(load_pc);
        valid_[i]      = true;
        store_pc_[i]   = store_pc;
    }

    void retiredClean(uint64_t, bool) override {}

    void reset() override
    {
        std::fill(valid_.begin(), valid_.end(), false);
        std::fill(store_pc_.begin(), store_pc_.end(), static_cast<uint64_t>(0));
    }

    const char* name() const override { return "store_pc"; }

private:
    size_t index(uint64_t pc) const { return static_cast<size_t>((pc >> 2) & mask_); }

    const uint64_t        mask_;
    std::vector<bool>     valid_;
    std::vector<uint64_t> store_pc_;
};

// ---------------------------------------------------------------------------
// No prediction at all: every load goes past every unknown store.
// ---------------------------------------------------------------------------
class VanadisMemDepNone : public VanadisMemDepPredictor
{
public:
    bool        speculate(uint64_t, const VanadisStoreQView&) override { return true; }
    void        violated(uint64_t, uint64_t) override {}
    void        retiredClean(uint64_t, bool) override {}
    void        reset() override {}
    const char* name() const override { return "none"; }
};

// Never speculate: a load waits for every older store whose address is unknown.
// Not one of the two designs -- it is the control case, the configuration in
// which a load reorders only against stores that are already resolved and
// proved not to touch it.
class VanadisMemDepHold : public VanadisMemDepPredictor
{
public:
    bool        speculate(uint64_t, const VanadisStoreQView&) override { return false; }
    void        violated(uint64_t, uint64_t) override {}
    void        retiredClean(uint64_t, bool) override {}
    void        reset() override {}
    const char* name() const override { return "hold"; }
};

/// Round up to a power of two, so that the index can be a mask.
inline size_t
vanadisMemDepRoundUp(size_t n)
{
    size_t p = 1;
    while ( p < n ) { p <<= 1; }
    return p;
}

/// The named predictor, or nullptr when the name is not one of the three.
inline VanadisMemDepPredictor*
vanadisMakeMemDepPredictor(const std::string& kind, size_t counter_entries, size_t store_entries, bool decay_spec_only)
{
    if ( kind == "counter" ) { return new VanadisMemDepCounter(vanadisMemDepRoundUp(counter_entries), decay_spec_only); }
    if ( kind == "store_pc" ) { return new VanadisMemDepStorePC(vanadisMemDepRoundUp(store_entries)); }
    if ( kind == "none" ) { return new VanadisMemDepNone(); }
    if ( kind == "hold" ) { return new VanadisMemDepHold(); }
    return nullptr;
}

} // namespace Vanadis
} // namespace SST

#endif
