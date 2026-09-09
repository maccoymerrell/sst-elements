// Copyright 2009-2025 NTESS. Under the terms
// of Contract DE-NA0003525 with NTESS, the U.S.
// Government retains certain rights in this software.
//
// Copyright (c) 2009-2025, NTESS
// All rights reserved.
//
// This file is part of the SST software package. For license
// information, see the LICENSE file in the top level directory of the
// distribution.

#ifndef _H_VANADIS_TLB
#define _H_VANADIS_TLB

#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

#include <sst/core/interfaces/stdMem.h>
#include <sst/core/output.h>

namespace SST {
namespace Vanadis {

// ADDRESS TRANSLATION HAS A COST.
//
// Until this file existed the core translated for free: the component in front
// of the caches rewrote an address and nothing was charged for it, so a program
// whose working set spanned ten thousand pages paid exactly what one spanning
// one page paid. Every reference core spends real cycles here -- a small
// first-level buffer per side, a larger one shared between them, and a hardware
// walker that reads the page table THROUGH THE CACHES when both miss.
//
// What is modelled, and what is not. This unit decides WHEN a translation is
// available and what it costs to obtain; it never decides WHAT the translation
// is. The address a request finally carries is still produced by the same
// functional path as before, so no program's result can change here. A walk's
// price is the price of its memory accesses: real reads, of real page-table
// addresses, that traverse the real cache hierarchy, compete for its ports and
// evict its lines. Their contents are discarded.
//
// The addresses walked are the addresses a RISC-V Sv39 table would be read at
// for this virtual page -- one entry from a root page, one from a second-level
// page chosen by the top index, one from a leaf page chosen by the top two --
// so the walk has the locality a real one has: the root is always cached, the
// second level nearly always, the leaf rarely.

// One set-associative (or fully associative, when ways == entries) buffer of
// virtual page numbers. Least-recently-used replacement.
class VanadisTLBArray
{
public:
    VanadisTLBArray() : sets_(0), ways_(0), clock_(0) {}

    void configure(const uint32_t entries, const uint32_t ways)
    {
        ways_ = (ways == 0) ? 1 : ways;
        if ( ways_ > entries ) { ways_ = entries; }
        sets_ = (entries == 0) ? 0 : (entries / ways_);
        if ( sets_ == 0 ) { sets_ = 1; }

        vpn_.assign(static_cast<size_t>(sets_) * ways_, 0);
        age_.assign(static_cast<size_t>(sets_) * ways_, 0);
        valid_.assign(static_cast<size_t>(sets_) * ways_, 0);
        clock_ = 0;
    }

    uint32_t entries() const { return sets_ * ways_; }

    bool lookup(const uint64_t vpn)
    {
        if ( 0 == sets_ ) { return false; }
        const size_t base = static_cast<size_t>(vpn % sets_) * ways_;
        for ( uint32_t w = 0; w < ways_; ++w ) {
            if ( valid_[base + w] && (vpn_[base + w] == vpn) ) {
                age_[base + w] = ++clock_;
                return true;
            }
        }
        return false;
    }

    void insert(const uint64_t vpn)
    {
        if ( 0 == sets_ ) { return; }
        const size_t base = static_cast<size_t>(vpn % sets_) * ways_;

        size_t   victim   = base;
        uint64_t best_age = UINT64_MAX;

        for ( uint32_t w = 0; w < ways_; ++w ) {
            if ( valid_[base + w] && (vpn_[base + w] == vpn) ) {
                age_[base + w] = ++clock_;
                return;
            }
            if ( !valid_[base + w] ) { victim = base + w; best_age = 0; break; }
            if ( age_[base + w] < best_age ) { best_age = age_[base + w]; victim = base + w; }
        }

        vpn_[victim]   = vpn;
        valid_[victim] = 1;
        age_[victim]   = ++clock_;
    }

    void clear()
    {
        std::fill(valid_.begin(), valid_.end(), 0);
        clock_ = 0;
    }

private:
    std::vector<uint64_t> vpn_;
    std::vector<uint64_t> age_;
    std::vector<uint8_t>  valid_;
    uint32_t              sets_;
    uint32_t              ways_;
    uint64_t              clock_;
};

// The host's whole translation path: an instruction-side and a data-side
// first-level buffer, one second level shared between them, and the walker.
class VanadisTLBUnit
{
public:
    VanadisTLBUnit() :
        enabled_(false), mem_(nullptr), output_(nullptr), cycle_(0),
        l1_hit_cycles_(0), l2_hit_cycles_(5), levels_(3), max_walkers_(2),
        page_shift_(12), root_(0), arena_pages_(4096), active_walks_(0),
        l1i_hits_(0), l1i_misses_(0), l1d_hits_(0), l1d_misses_(0),
        l2_hits_(0), l2_misses_(0), walks_(0), walk_reads_(0), walk_declined_(0)
    {}

    void configure(
        const bool enabled, const uint32_t l1i_entries, const uint32_t l1i_ways, const uint32_t l1d_entries,
        const uint32_t l1d_ways, const uint32_t l2_entries, const uint32_t l2_ways, const uint16_t l1_hit_cycles,
        const uint16_t l2_hit_cycles, const uint16_t levels, const uint16_t walkers, const uint64_t page_size,
        const uint64_t root, const uint64_t arena_pages)
    {
        enabled_       = enabled;
        l1_hit_cycles_ = l1_hit_cycles;
        l2_hit_cycles_ = l2_hit_cycles;
        levels_        = (levels == 0) ? 1 : levels;
        max_walkers_   = (walkers == 0) ? 1 : walkers;
        root_          = root;
        arena_pages_   = (arena_pages == 0) ? 1 : arena_pages;

        page_shift_ = 0;
        while ( (static_cast<uint64_t>(1) << page_shift_) < page_size ) { ++page_shift_; }

        l1i_.configure(l1i_entries, l1i_ways);
        l1d_.configure(l1d_entries, l1d_ways);
        l2_.configure(l2_entries, l2_ways);
    }

    void setInterface(SST::Interfaces::StandardMem* m) { mem_ = m; }
    void setOutput(SST::Output* o) { output_ = o; }

    bool enabled() const { return enabled_; }
    uint64_t currentCycle() const { return cycle_; }

    // Ask for the translation of this address. True means the access may go on
    // this cycle; false means the caller must ask again on a later cycle, and
    // whatever work is needed -- a second-level lookup, a page-table walk -- has
    // been started.
    bool access(const bool instruction, const uint64_t vaddr)
    {
        if ( !enabled_ ) { return true; }

        const uint64_t vpn = vaddr >> page_shift_;

        VanadisTLBArray& l1 = instruction ? l1i_ : l1d_;

        if ( l1.lookup(vpn) ) {
            if ( instruction ) { l1i_hits_++; } else { l1d_hits_++; }
            return (0 == l1_hit_cycles_);
        }

        auto pending = pending_.find(vpn);
        if ( pending != pending_.end() ) {
            // Somebody -- possibly the other side of the machine -- is already
            // obtaining this translation. Wait with them.
            if ( instruction ) { pending->second.for_inst = true; }
            else               { pending->second.for_data = true; }
            return false;
        }

        if ( instruction ) { l1i_misses_++; } else { l1d_misses_++; }

        Pending p;
        p.for_inst    = instruction;
        p.for_data    = !instruction;
        p.walking     = false;
        p.level       = 0;
        p.ready_cycle = 0;

        if ( l2_.lookup(vpn) ) {
            l2_hits_++;
            p.ready_cycle = cycle_ + l2_hit_cycles_;
            pending_.emplace(vpn, p);
            return false;
        }

        l2_misses_++;

        if ( active_walks_ >= max_walkers_ ) {
            // Every walker is busy. Nothing is recorded, so the next cycle asks
            // again and this miss is not counted twice.
            walk_declined_++;
            l2_misses_--;
            if ( instruction ) { l1i_misses_--; } else { l1d_misses_--; }
            return false;
        }

        p.walking = true;
        pending_.emplace(vpn, p);
        active_walks_++;
        walks_++;
        issueWalkRead(vpn, 0);
        return false;
    }

    // One core cycle. Translations whose cost has been paid become usable.
    void tick(const uint64_t cycle)
    {
        cycle_ = cycle;
        if ( !enabled_ || pending_.empty() ) { return; }

        for ( auto itr = pending_.begin(); itr != pending_.end(); ) {
            if ( itr->second.walking || (itr->second.ready_cycle > cycle_) ) { ++itr; continue; }

            if ( itr->second.for_inst ) { l1i_.insert(itr->first); }
            if ( itr->second.for_data ) { l1d_.insert(itr->first); }
            itr = pending_.erase(itr);
        }
    }

    // A read this walker issued has come back. True when the response belonged
    // to a walk and has been consumed; the caller must then not look for it
    // among its own outstanding loads.
    bool handleResponse(const SST::Interfaces::StandardMem::Request::id_t id)
    {
        if ( walk_ids_.empty() ) { return false; }

        auto itr = walk_ids_.find(id);
        if ( itr == walk_ids_.end() ) { return false; }

        const uint64_t vpn = itr->second;
        walk_ids_.erase(itr);

        auto p = pending_.find(vpn);
        if ( p == pending_.end() ) {
            // The walk's requester went away with a pipeline flush.
            if ( active_walks_ > 0 ) { active_walks_--; }
            return true;
        }

        p->second.level++;

        if ( p->second.level < levels_ ) {
            issueWalkRead(vpn, p->second.level);
            return true;
        }

        // The walk is finished: the translation is in the second level now and
        // reaches the first level on the next cycle.
        l2_.insert(vpn);
        p->second.walking     = false;
        p->second.ready_cycle = cycle_;
        if ( active_walks_ > 0 ) { active_walks_--; }
        return true;
    }

    // Statistics, read out by whoever owns this unit.
    uint64_t l1iHits() const { return l1i_hits_; }
    uint64_t l1iMisses() const { return l1i_misses_; }
    uint64_t l1dHits() const { return l1d_hits_; }
    uint64_t l1dMisses() const { return l1d_misses_; }
    uint64_t l2Hits() const { return l2_hits_; }
    uint64_t l2Misses() const { return l2_misses_; }
    uint64_t walks() const { return walks_; }
    uint64_t walkReads() const { return walk_reads_; }
    uint64_t walksDeclined() const { return walk_declined_; }
    size_t   outstandingWalks() const { return active_walks_; }
    size_t   pendingCount() const { return pending_.size(); }

private:
    struct Pending {
        bool     for_inst;
        bool     for_data;
        bool     walking;
        uint16_t level;
        uint64_t ready_cycle;
    };

    // Where the level-`level` entry for this virtual page lives, laid out as a
    // RISC-V Sv39 table would lay it out: a root page, a second-level page per
    // top index, and a leaf page per (top, middle) pair. The whole arena is
    // wrapped into `arena_pages` pages so that it cannot run off the end of the
    // modelled physical memory whatever the address is.
    uint64_t walkAddress(const uint64_t vpn, const uint16_t level) const
    {
        const uint64_t idx2 = (vpn >> 18) & 0x1FF;
        const uint64_t idx1 = (vpn >> 9) & 0x1FF;
        const uint64_t idx0 = vpn & 0x1FF;

        uint64_t page   = 0;
        uint64_t offset = 0;

        if ( 0 == level ) {
            page   = 0;
            offset = idx2 * 8;
        }
        else if ( 1 == level ) {
            page   = 1 + idx2;
            offset = idx1 * 8;
        }
        else {
            page   = 1 + 512 + (idx2 * 512) + idx1;
            offset = idx0 * 8;
        }

        return root_ + ((page % arena_pages_) << page_shift_) + offset;
    }

    void issueWalkRead(const uint64_t vpn, const uint16_t level)
    {
        if ( nullptr == mem_ ) { return; }

        SST::Interfaces::StandardMem::Read* req =
            new SST::Interfaces::StandardMem::Read(walkAddress(vpn, level), 8);
        walk_ids_[req->getID()] = vpn;
        walk_reads_++;
        mem_->send(req);
    }

    bool                          enabled_;
    SST::Interfaces::StandardMem* mem_;
    SST::Output*                  output_;

    VanadisTLBArray l1i_;
    VanadisTLBArray l1d_;
    VanadisTLBArray l2_;

    std::unordered_map<uint64_t, Pending>                                          pending_;
    std::unordered_map<SST::Interfaces::StandardMem::Request::id_t, uint64_t>      walk_ids_;

    uint64_t cycle_;
    uint16_t l1_hit_cycles_;
    uint16_t l2_hit_cycles_;
    uint16_t levels_;
    uint16_t max_walkers_;
    uint16_t page_shift_;
    uint64_t root_;
    uint64_t arena_pages_;
    size_t   active_walks_;

    uint64_t l1i_hits_;
    uint64_t l1i_misses_;
    uint64_t l1d_hits_;
    uint64_t l1d_misses_;
    uint64_t l2_hits_;
    uint64_t l2_misses_;
    uint64_t walks_;
    uint64_t walk_reads_;
    uint64_t walk_declined_;
};

} // namespace Vanadis
} // namespace SST

#endif
