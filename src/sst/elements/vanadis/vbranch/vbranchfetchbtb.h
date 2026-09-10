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

#ifndef _H_VANADIS_BRANCH_FETCH_BTB
#define _H_VANADIS_BRANCH_FETCH_BTB

// THE FETCH-STAGE BRANCH TARGET BUFFER.
//
// This is the structure a real processor's fetch stage reads, and the only one
// it reads. It is indexed by an ALIGNED INSTRUCTION FETCH BLOCK -- here the
// 64-byte block that is also the instruction cache's line -- and one entry
// describes the whole block:
//
//   * a BITMAP with one bit per instruction slot in the block, saying which
//     slots hold a branch. RISC-V instructions are 2-byte aligned, so a
//     64-byte block has 32 slots and the bitmap is 32 bits;
//   * for each marked slot, the branch's class (conditional, direct jump,
//     direct call, indirect jump, indirect call, return), its width, its
//     target, and TWO BITS OF DIRECTION: the prediction bit and hysteresis bit
//     of a bimodal counter. Those two bits are the base predictor of the
//     core's TAGE-SC-L, kept here rather than in a table of its own, so that a
//     single lookup gives the fetch stage both where the branch goes and
//     whether to go there.
//
// A fetch reads the entry for the block its address falls in, finds the first
// marked slot at or after the fetch offset, and either redirects to that
// branch's target or runs to the end of the block. Entries are ALLOCATED WHEN
// A BRANCH RESOLVES that the buffer did not know about, which is why a
// branch's first execution always predicts fall-through.
//
// SOURCES. AMD, "Software Optimization Guide for the AMD Zen4
// Microarchitecture", publication 57647 rev 1.00, April 2023, section 2.8:
// "The branch target buffer (BTB) is a two-level structure accessed using the
// fetch address of the previous fetch block. Each BTB entry includes
// information for branches and their targets. Each BTB entry can hold up to
// two branches" -- both in "the same 64 byte aligned cacheline". "The L1 BTB
// has 1536 entries and predicts with zero prediction bubbles for conditional
// and unconditional direct branches, and one cycle bubble for calls, returns
// and indirect branches. The L2 BTB has 7680 entries and creates three
// prediction bubbles if its prediction differs from that of the L1 BTB."
// Section 2.8.1.5: "Conditional branches that have not yet been discovered to
// be taken are not marked in the BTBs. These branches are implicitly predicted
// not-taken." Section 2.8.1.6 describes a fetch window tracking structure
// whose entry "holds branch prediction information for up to a full 64-byte
// cache line".
//
// Reinman, Calder and Austin, "Optimizations Enabled by a Decoupled Front-End
// Architecture", UCSD CS2000-0645: "A BTB entry holds the taken target address
// for a branch along with other information, such as the type of the branch,
// conditional branch prediction information, and possibly the fall-through
// address of the branch", and of their own fetch target buffer's direction
// bits, "a 2-bit meta-chooser and a 2-bit bimodal predictor, both stored in
// the branch predictor entry with their corresponding branch". That is the
// arrangement copied here: the bimodal counter lives in the buffer entry, per
// branch.
//
// Yeh and Patt's basic block target buffer, described in the same paper, is
// the ancestor of the block-indexed organisation; Perais and Sheikh, "Branch
// Target Buffer Organizations", MICRO-56, 2023, is the modern survey of
// instruction-, block- and region-indexed buffers.

#include "vbranch/vbranchcheckpoint.h"

#include <cstdint>
#include <list>
#include <unordered_map>
#include <vector>

namespace SST {
namespace Vanadis {

class VanadisFetchBTB
{
public:
    // The most branches one entry describes. AMD's entry holds two, sharing
    // one set of target bits; this organisation gives every marked slot its
    // own target, and four covers a 64-byte block of RV64 code with room to
    // spare. `slotOverflow()` counts the times it did not, so the choice is
    // checkable rather than assumed.
    enum : uint32_t { MAX_SLOTS = 4 };

    struct Slot {
        uint64_t           target     = 0;
        VanadisBranchClass cls        = VanadisBranchClass::CONDITIONAL;
        uint8_t            offset     = 0;  // byte offset of the branch inside the block
        uint8_t            width      = 4;
        bool               has_target = false;
        int8_t             pred       = 0;  // bimodal prediction bit
        int8_t             hyst       = 1;  // bimodal hysteresis bit
    };

    struct Lookup {
        bool     block_hit = false;  // the block has an entry
        bool     found     = false;  // and a marked branch at or after the offset
        bool     from_l2   = false;  // the entry came from the second level
        uint64_t branch_pc = 0;
        Slot     slot;
    };

    VanadisFetchBTB(const uint32_t l1_entries, const uint32_t l2_entries, const uint32_t block_bytes) :
        l1_(l1_entries == 0 ? 1 : l1_entries),
        l2_(l2_entries),
        block_bytes_(block_bytes),
        block_mask_(~(static_cast<uint64_t>(block_bytes) - 1))
    {}

    uint64_t blockOf(const uint64_t pc) const { return pc & block_mask_; }
    uint64_t blockBytes() const { return block_bytes_; }

    // ---- the fetch-stage lookup -------------------------------------------
    //
    // One access. The block's entry is read, the bitmap is scanned from the
    // fetch offset for the first marked slot, and that slot's payload comes
    // back with it.
    Lookup lookup(const uint64_t pc)
    {
        Lookup    out;
        const uint64_t block  = blockOf(pc);
        const uint32_t offset = static_cast<uint32_t>(pc - block);

        Entry* e = find(l1_, block);
        if ( nullptr != e ) { ++l1_hits_; }
        else {
            e = find(l2_, block);
            if ( nullptr == e ) {
                ++misses_;
                return out;
            }
            // A second-level hit is promoted into the first level, and what it
            // displaces goes back to the second. That is what makes the two
            // levels a hierarchy rather than two independent buffers.
            ++l2_hits_;
            out.from_l2 = true;
            Entry copy  = *e;
            erase(l2_, block);
            insertPromote(copy);
            e = find(l1_, block);
            if ( nullptr == e ) { return out; }
        }

        out.block_hit = true;

        const uint32_t first = offset >> 1;                 // 2-byte instruction slots
        const uint32_t mask  = (first >= 32) ? 0u : (0xFFFFFFFFu << first);
        const uint32_t bits  = e->bitmap & mask;
        if ( 0 == bits ) { return out; }

        const uint32_t slot_index = static_cast<uint32_t>(__builtin_ctz(bits));
        const uint8_t  want       = static_cast<uint8_t>(slot_index << 1);

        for ( uint32_t i = 0; i < e->count; ++i ) {
            if ( e->slot[i].offset == want ) {
                out.found     = true;
                out.slot      = e->slot[i];
                out.branch_pc = block + want;
                return out;
            }
        }

        return out;
    }

    // The entry for one branch, by its own address. The decode stage asks for
    // this when the fetch stage did not reach the branch, and the resolution
    // path asks for it to train.
    Slot* slotFor(const uint64_t pc)
    {
        const uint64_t block  = blockOf(pc);
        const uint8_t  offset = static_cast<uint8_t>(pc - block);

        Entry* e = find(l1_, block);
        if ( nullptr == e ) { e = find(l2_, block); }
        if ( nullptr == e ) { return nullptr; }

        for ( uint32_t i = 0; i < e->count; ++i ) {
            if ( e->slot[i].offset == offset ) { return &e->slot[i]; }
        }
        return nullptr;
    }

    // ---- allocation, at resolution ----------------------------------------
    //
    // A branch the buffer did not know about is marked here, and nowhere else.
    // A conditional branch is marked only once it has resolved taken, which is
    // the rule the Zen 4 optimisation guide states: a conditional that has
    // never been taken is not in the buffer and is implicitly predicted
    // not-taken, and costs nothing to hold.
    void allocate(
        const uint64_t pc, const VanadisBranchClass cls, const uint8_t width, const uint64_t target,
        const bool has_target, const bool taken)
    {
        if ( vanadisBranchIsConditional(cls) && !taken ) { return; }

        const uint64_t block  = blockOf(pc);
        const uint8_t  offset = static_cast<uint8_t>(pc - block);

        Entry* e = find(l1_, block);
        if ( nullptr == e ) {
            Entry* from_l2 = find(l2_, block);
            if ( nullptr != from_l2 ) {
                Entry copy = *from_l2;
                erase(l2_, block);
                insertPromote(copy);
            }
            else {
                Entry fresh;
                fresh.block = block;
                insertPromote(fresh);
            }
            e = find(l1_, block);
            if ( nullptr == e ) { return; }
        }

        for ( uint32_t i = 0; i < e->count; ++i ) {
            if ( e->slot[i].offset == offset ) { return; }
        }

        if ( e->count >= MAX_SLOTS ) {
            // The entry describes as many branches as it can. The oldest of
            // them goes, which is the replacement a hardware entry with a
            // fixed number of branch fields has to make.
            ++slot_overflow_;
            e->bitmap &= ~(1u << (e->slot[0].offset >> 1));
            for ( uint32_t i = 1; i < e->count; ++i ) { e->slot[i - 1] = e->slot[i]; }
            --e->count;
        }

        Slot s;
        s.offset     = offset;
        s.width      = width;
        s.cls        = cls;
        s.target     = target;
        s.has_target = has_target;
        // A branch is marked because it was taken, so its bimodal counter
        // starts weakly taken. An unconditional branch's counter is never read.
        s.pred       = 1;
        s.hyst       = 0;

        e->slot[e->count++] = s;
        e->bitmap |= (1u << (offset >> 1));
        ++allocations_;
    }

    void setTarget(const uint64_t pc, const uint64_t target)
    {
        Slot* s = slotFor(pc);
        if ( nullptr == s ) { return; }
        if ( s->has_target && (s->target != target) ) { ++target_changes_; }
        s->target     = target;
        s->has_target = true;
    }

    void setBimodal(const uint64_t pc, const int8_t pred, const int8_t hyst)
    {
        Slot* s = slotFor(pc);
        if ( nullptr == s ) { return; }
        s->pred = pred;
        s->hyst = hyst;
    }

    // ---- counters ---------------------------------------------------------
    uint64_t l1Hits() const { return l1_hits_; }
    uint64_t l2Hits() const { return l2_hits_; }
    uint64_t misses() const { return misses_; }
    uint64_t allocations() const { return allocations_; }
    uint64_t l1Evictions() const { return l1_evictions_; }
    uint64_t l2Evictions() const { return l2_evictions_; }
    uint64_t slotOverflow() const { return slot_overflow_; }
    uint64_t targetChanges() const { return target_changes_; }

private:
    struct Entry {
        uint64_t block  = 0;
        uint32_t bitmap = 0;
        uint32_t count  = 0;
        Slot     slot[MAX_SLOTS];
    };

    // One level: a capacity model with least-recently-used replacement. It
    // models how many blocks the level holds, not how they are placed in sets;
    // an associativity model would need a real index function and is a
    // different question from the one this front end is being measured on.
    struct Level {
        explicit Level(const uint32_t cap) : capacity(cap) {}
        uint32_t                                                     capacity;
        std::unordered_map<uint64_t, Entry>                          map;
        std::list<uint64_t>                                          lru;
        std::unordered_map<uint64_t, std::list<uint64_t>::iterator>  pos;
    };

    static Entry* find(Level& lv, const uint64_t block)
    {
        auto itr = lv.map.find(block);
        if ( itr == lv.map.end() ) { return nullptr; }
        auto p = lv.pos.find(block);
        if ( p != lv.pos.end() ) {
            lv.lru.splice(lv.lru.begin(), lv.lru, p->second);
            p->second = lv.lru.begin();
        }
        return &itr->second;
    }

    static void erase(Level& lv, const uint64_t block)
    {
        auto p = lv.pos.find(block);
        if ( p != lv.pos.end() ) {
            lv.lru.erase(p->second);
            lv.pos.erase(p);
        }
        lv.map.erase(block);
    }

    // Put an entry in the first level, sending whatever it displaces to the
    // second, and dropping whatever the second displaces.
    void insertPromote(const Entry& e)
    {
        if ( (l1_.capacity > 0) && (l1_.map.size() >= l1_.capacity) ) {
            const uint64_t victim = l1_.lru.back();
            auto           v      = l1_.map.find(victim);
            if ( v != l1_.map.end() ) {
                Entry demoted = v->second;
                erase(l1_, victim);
                ++l1_evictions_;
                insertLevel(l2_, demoted, &l2_evictions_);
            }
        }
        insertLevel(l1_, e, &l1_evictions_);
    }

    static void insertLevel(Level& lv, const Entry& e, uint64_t* evictions)
    {
        if ( 0 == lv.capacity ) { return; }
        if ( lv.map.size() >= lv.capacity ) {
            const uint64_t victim = lv.lru.back();
            erase(lv, victim);
            if ( nullptr != evictions ) { ++(*evictions); }
        }
        lv.map[e.block] = e;
        lv.lru.push_front(e.block);
        lv.pos[e.block] = lv.lru.begin();
    }

    Level          l1_;
    Level          l2_;
    const uint64_t block_bytes_;
    const uint64_t block_mask_;

    uint64_t l1_hits_        = 0;
    uint64_t l2_hits_        = 0;
    uint64_t misses_         = 0;
    uint64_t allocations_    = 0;
    uint64_t l1_evictions_   = 0;
    uint64_t l2_evictions_   = 0;
    uint64_t slot_overflow_  = 0;
    uint64_t target_changes_ = 0;
};

} // namespace Vanadis
} // namespace SST

#endif
