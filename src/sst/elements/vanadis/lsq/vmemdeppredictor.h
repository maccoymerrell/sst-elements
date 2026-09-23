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
//             of them, at the cost of a 64-bit field instead of two bits. Its
//             table is invalidated every `mdp_clear_interval` cycles, because
//             an entry armed by one flush has no other way to be taken back.
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

    /// THE HOLD THAT BOUGHT NOTHING. The load this predictor held reached the
    /// end of the older stores with none of them overlapping it. Evidence
    /// against the entry that held it, and the only evidence a held load can
    /// produce: a predictor without this has no way to take an entry back.
    virtual void heldNeedlessly(uint64_t) {}

    /// THE PATH THE LOAD WAS REACHED ALONG. Called in program order for every
    /// memory instruction as it enters the queues, so that a predictor which
    /// distinguishes contexts has something to distinguish them by. A predictor
    /// indexed by the load address alone ignores it.
    virtual void pathUpdate(uint64_t) {}

    /// WHAT THE TABLE DID, for the statistics. `predictions` counts the times
    /// the table was consulted and had something to say; `correct` the
    /// speculations that retired without a violation; `violations` the loads
    /// that read bytes an older store then wrote; `replays` the entries the
    /// table allocated or rewrote in response; `occupancy` the entries it is
    /// holding now.
    virtual uint64_t predictions() const { return predictions_; }
    virtual uint64_t correct() const { return correct_; }
    virtual uint64_t violations() const { return violations_; }
    virtual uint64_t replays() const { return replays_; }
    virtual uint64_t occupancy() const { return 0; }
    virtual uint64_t capacity() const { return 0; }

    /// One core cycle has passed. Returns true on the cycle the table was
    /// invalidated, so that the queue can count it. A predictor whose entries
    /// carry their own evidence -- the counter's, which decays -- does not age
    /// on the clock and returns false for ever.
    virtual bool tick(uint64_t) { return false; }

    virtual const char* name() const = 0;

protected:
    uint64_t predictions_ = 0;
    uint64_t correct_     = 0;
    uint64_t violations_  = 0;
    uint64_t replays_     = 0;
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
// An entry is armed by ONE flush and, on its own, is never taken back: the hold
// is conditional on the named store being one of the ones ahead of this load,
// so an entry that has stopped describing the program was expected to cost
// nothing the moment that store stopped appearing. In a loop where that store
// is nearly always in flight and nearly never aliases, it costs a hold on every
// execution instead, and there is no evidence a load can produce that will take
// the entry back.
//
// PERIODIC CLEAR. The whole table is invalidated every `clear_interval` cycles,
// which is how a store-set predictor drops what it has learned wrongly
// (Chrysos and Emer, "Memory Dependence Prediction using Store Sets", ISCA
// 1998, whose SSIT is invalidated on a periodic counter; gem5's descendant of
// it clears every 250,000 load/store instructions, `store_set_clear_period`).
// A load whose dependence is real is re-armed by its next flush, at the price
// of that one flush per interval; a load armed by an accident is not. Zero
// means never, which is the behaviour before this parameter existed.
class VanadisMemDepStorePC : public VanadisMemDepPredictor
{
public:
    VanadisMemDepStorePC(size_t entries, uint64_t clear_interval) :
        mask_(entries - 1), clear_interval_(clear_interval), last_clear_(0), valid_(entries, false),
        store_pc_(entries, 0)
    {}

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

    /// Invalidating means clearing the valid bits: a store address is read only
    /// through one, so the addresses themselves need not be touched.
    bool tick(uint64_t cycle) override
    {
        if ( 0 == clear_interval_ ) { return false; }
        if ( (cycle - last_clear_) < clear_interval_ ) { return false; }
        last_clear_ = cycle;
        std::fill(valid_.begin(), valid_.end(), false);
        return true;
    }

    const char* name() const override { return "store_pc"; }

private:
    size_t index(uint64_t pc) const { return static_cast<size_t>((pc >> 2) & mask_); }

    const uint64_t        mask_;
    const uint64_t        clear_interval_;
    uint64_t              last_clear_;
    std::vector<bool>     valid_;
    std::vector<uint64_t> store_pc_;
};

// ---------------------------------------------------------------------------
// C. PHAST: the same load, reached along different paths, predicted separately.
// ---------------------------------------------------------------------------
//
// THE PROBLEM WITH B. A store-set predictor indexed by the load's address alone
// gives one answer for that load wherever it is reached from. A load inside a
// function called from two places, or inside a loop entered by two paths, has
// one dependence on one path and none on the other, and one entry cannot say
// both: whichever the last violation armed is what every execution gets, so the
// path that does not alias is held and the path that does is the only one
// predicted right.
//
// WHAT PHAST DOES. It indexes on the load's address AND the path taken to reach
// it, at several history lengths at once, and lets the longest length that has
// an entry for this context answer. The published design is Kim and Ros,
// "Effective Context-Sensitive Memory Dependence Prediction", HPCA 2024: for
// each load it finds the shortest history that predicts it precisely, using a
// TAGE-like set of components at a geometric series of history lengths, trained
// on the execution path between the conflicting store and the load.
//
// WHAT IS IMPLEMENTED HERE, and where it departs from that paper -- stated
// because a model that quietly differs from what it cites is worse than one
// that cites nothing:
//
//   * The COMPONENTS are as published: a PC-indexed base plus tagged
//     components at a geometric series of history lengths, longest match
//     provides, allocation on a misprediction goes to a longer component than
//     the one that provided, and a usefulness counter protects entries from
//     being taken.
//   * The INDEX is the gshare-style fold the owner named: the load address
//     hashed with the folded path history of that component's length. The
//     paper's hash is more elaborate.
//   * WHAT AN ENTRY HOLDS is the ADDRESS OF THE CONFLICTING STORE, as
//     predictor B holds, rather than the paper's distance to that store along
//     the path. The reason is the interface: the queue tells a predictor
//     whether a named store instruction is among the unresolved older ones, and
//     nothing about distances. ASSUMED: that the context-sensitivity, and not
//     the distance encoding, is what the prediction accuracy comes from. The
//     distance form would need the queue to expose its order, which is a change
//     to the queue and not to the predictor.
//   * THE PATH HISTORY is built from the MEMORY INSTRUCTIONS in program order,
//     two bits of each address shifted in as it enters the queues, rather than
//     from branch outcomes. ASSUMED: that this distinguishes the paths that
//     matter here, since two paths to one load that differ at all differ in the
//     loads and stores they execute. The queue does not see branches; taking
//     the history from the branch unit would be a second change in a second
//     component and is the better form if this one proves too coarse.
//
class VanadisMemDepPhast : public VanadisMemDepPredictor
{
public:
    // The history lengths, in bits of the path register, geometric as the
    // published design's are. The base component has no history at all.
    static constexpr size_t COMPONENTS = 4;

    VanadisMemDepPhast(size_t base_entries, size_t comp_entries, uint64_t clear_interval) :
        base_mask_(base_entries - 1), comp_mask_(comp_entries - 1), clear_interval_(clear_interval),
        last_clear_(0), base_(base_entries), hist_{ 4, 12, 36, 108 }
    {
        for ( size_t c = 0; c < COMPONENTS; ++c ) { comp_[c].assign(comp_entries, Entry()); }
    }

    void pathUpdate(uint64_t pc) override
    {
        // Two bits of each memory instruction's address, shifted into a 128-bit
        // register in program order. Two bits rather than one because one bit
        // per instruction makes two different addresses agree half the time.
        const uint64_t bits = ((pc >> 2) ^ (pc >> 5) ^ (pc >> 9) ^ (pc >> 13)) & 0x3;
        phr_hi_             = (phr_hi_ << 2) | (phr_lo_ >> 62);
        phr_lo_             = (phr_lo_ << 2) | bits;
    }

    bool speculate(uint64_t load_pc, const VanadisStoreQView& older) override
    {
        const Provider p = provider(load_pc);
        if ( !p.found ) { return true; }        // nothing known: let it go
        predictions_++;
        return !older.containsStorePC(p.store_pc);
    }

    void violated(uint64_t load_pc, uint64_t store_pc) override
    {
        violations_++;
        const Provider p = provider(load_pc);

        // The component that answered had the right store and the load
        // violated anyway -- the store was not among the unresolved ones when
        // the load was asked. Nothing to learn; strengthen what is there.
        if ( p.found && p.store_pc == store_pc ) {
            if ( p.comp < COMPONENTS ) {
                Entry& e = comp_[p.comp][compIndex(load_pc, p.comp)];
                if ( e.conf < 3 ) { e.conf++; }
            }
            return;
        }

        // Otherwise allocate in a component with a LONGER history than the one
        // that provided, which is what makes the predictor find the shortest
        // history that separates the two contexts. A base-only load allocates
        // in the shortest component.
        const size_t from  = p.found && p.comp < COMPONENTS ? p.comp + 1 : 0;
        bool         taken = false;
        for ( size_t c = from; c < COMPONENTS && !taken; ++c ) {
            Entry& e = comp_[c][compIndex(load_pc, c)];
            if ( e.valid && e.useful > 0 ) { e.useful--; continue; }   // protected; age it
            e.valid    = true;
            e.tag      = compTag(load_pc, c);
            e.store_pc = store_pc;
            e.conf     = 3;
            e.useful   = 0;
            taken      = true;
            replays_++;
        }
        if ( !taken ) {
            // Every component that could hold it is protected. The base still
            // records the dependence, so the load is at least held somewhere.
            BaseEntry& b = base_[static_cast<size_t>((load_pc >> 2) & base_mask_)];
            b.valid      = true;
            b.store_pc   = store_pc;
            replays_++;
        }
    }

    /// The entry that held this load was wrong: nothing older overlapped it.
    /// Confidence falls, and an entry nothing confirms is given up, so a
    /// context whose dependence was an accident stops costing anything.
    void heldNeedlessly(uint64_t load_pc) override
    {
        const Provider p = provider(load_pc);
        if ( !p.found ) { return; }
        if ( p.comp < COMPONENTS ) {
            Entry& e = comp_[p.comp][compIndex(load_pc, p.comp)];
            if ( e.conf > 0 ) { e.conf--; }
            if ( 0 == e.conf ) { e.valid = false; }
            if ( e.useful > 0 ) { e.useful--; }
        }
        else {
            base_[static_cast<size_t>((load_pc >> 2) & base_mask_)].valid = false;
        }
    }

    void retiredClean(uint64_t load_pc, bool speculated) override
    {
        if ( !speculated ) { return; }
        correct_++;
        const Provider p = provider(load_pc);
        if ( p.found && p.comp < COMPONENTS ) {
            Entry& e = comp_[p.comp][compIndex(load_pc, p.comp)];
            if ( e.useful < 3 ) { e.useful++; }
        }
    }

    void reset() override
    {
        for ( auto& b : base_ ) { b = BaseEntry(); }
        for ( size_t c = 0; c < COMPONENTS; ++c ) { std::fill(comp_[c].begin(), comp_[c].end(), Entry()); }
        phr_lo_ = phr_hi_ = 0;
    }

    // PERIODIC AGEING, not a wipe. A tagged component whose entries were all
    // cleared on a timer would relearn the same contexts from scratch every
    // interval and the long histories would never pay for themselves. What the
    // interval does here is halve the usefulness counters, so an entry nothing
    // has confirmed lately becomes available to be taken, and clear the base --
    // which is the untagged part and the part that goes stale the way B's whole
    // table does.
    bool tick(uint64_t cycle) override
    {
        if ( 0 == clear_interval_ ) { return false; }
        if ( (cycle - last_clear_) < clear_interval_ ) { return false; }
        last_clear_ = cycle;
        for ( auto& b : base_ ) { b.valid = false; }
        for ( size_t c = 0; c < COMPONENTS; ++c ) {
            for ( auto& e : comp_[c] ) { e.useful >>= 1; }
        }
        return true;
    }

    uint64_t occupancy() const override
    {
        uint64_t n = 0;
        for ( const auto& b : base_ ) { if ( b.valid ) { n++; } }
        for ( size_t c = 0; c < COMPONENTS; ++c ) {
            for ( const auto& e : comp_[c] ) { if ( e.valid ) { n++; } }
        }
        return n;
    }

    uint64_t capacity() const override { return base_.size() + COMPONENTS * comp_[0].size(); }

    const char* name() const override { return "phast"; }

private:
    struct Entry
    {
        uint64_t store_pc = 0;
        uint16_t tag      = 0;
        uint8_t  conf     = 0;
        uint8_t  useful   = 0;
        bool     valid    = false;
    };

    struct BaseEntry
    {
        uint64_t store_pc = 0;
        bool     valid    = false;
    };

    struct Provider
    {
        bool     found    = false;
        size_t   comp     = COMPONENTS;   // COMPONENTS means "the base answered"
        uint64_t store_pc = 0;
    };

    /// The path history folded down to `bits` bits, as a gshare index folds a
    /// long global history into an index-sized value.
    uint64_t foldedHistory(size_t bits) const
    {
        uint64_t v = 0;
        for ( size_t b = 0; b < bits; ++b ) {
            const uint64_t bit = (b < 64) ? ((phr_lo_ >> b) & 1ULL) : ((phr_hi_ >> (b - 64)) & 1ULL);
            v ^= bit << (b % 16);
        }
        return v;
    }

    size_t compIndex(uint64_t load_pc, size_t c) const
    {
        const uint64_t h = foldedHistory(hist_[c]);
        return static_cast<size_t>(((load_pc >> 2) ^ h ^ (h << 3)) & comp_mask_);
    }

    uint16_t compTag(uint64_t load_pc, size_t c) const
    {
        const uint64_t h = foldedHistory(hist_[c]);
        return static_cast<uint16_t>(((load_pc >> 2) ^ (load_pc >> 14) ^ (h >> 1) ^ (h << 7)) & 0xFFFF);
    }

    /// The longest component holding this context, or the base, or nothing.
    Provider provider(uint64_t load_pc) const
    {
        for ( size_t c = COMPONENTS; c-- > 0; ) {
            const Entry& e = comp_[c][compIndex(load_pc, c)];
            if ( e.valid && e.tag == compTag(load_pc, c) ) { return Provider{ true, c, e.store_pc }; }
        }
        const BaseEntry& b = base_[static_cast<size_t>((load_pc >> 2) & base_mask_)];
        if ( b.valid ) { return Provider{ true, COMPONENTS, b.store_pc }; }
        return Provider{};
    }

    const uint64_t         base_mask_;
    const uint64_t         comp_mask_;
    const uint64_t         clear_interval_;
    uint64_t               last_clear_;
    std::vector<BaseEntry> base_;
    std::vector<Entry>     comp_[COMPONENTS];
    const size_t           hist_[COMPONENTS];
    uint64_t               phr_lo_ = 0;
    uint64_t               phr_hi_ = 0;
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
vanadisMakeMemDepPredictor(
    const std::string& kind, size_t counter_entries, size_t store_entries, bool decay_spec_only,
    uint64_t store_clear_interval)
{
    if ( kind == "counter" ) { return new VanadisMemDepCounter(vanadisMemDepRoundUp(counter_entries), decay_spec_only); }
    if ( kind == "store_pc" ) {
        return new VanadisMemDepStorePC(vanadisMemDepRoundUp(store_entries), store_clear_interval);
    }
    if ( kind == "phast" ) {
        return new VanadisMemDepPhast(
            vanadisMemDepRoundUp(counter_entries), vanadisMemDepRoundUp(store_entries), store_clear_interval);
    }
    if ( kind == "none" ) { return new VanadisMemDepNone(); }
    if ( kind == "hold" ) { return new VanadisMemDepHold(); }
    return nullptr;
}

} // namespace Vanadis
} // namespace SST

#endif
