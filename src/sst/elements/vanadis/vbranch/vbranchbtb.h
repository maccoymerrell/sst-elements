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

#ifndef _H_VANADIS_BRANCH_BTB
#define _H_VANADIS_BRANCH_BTB

#include <cstdint>
#include <list>
#include <unordered_map>

namespace SST {
namespace Vanadis {

// A branch target buffer: the address a branch last went to, kept for a fixed
// number of branches with least-recently-used replacement. This is the whole
// of the address-only predictor, and the direction predictors keep it because
// it is still the only answer for an indirect jump.
class VanadisBranchTargetBuffer
{
public:
    explicit VanadisBranchTargetBuffer(const uint32_t entries) : max_entries(entries == 0 ? 1 : entries) {}

    // Returns true when an entry had to be thrown out to make room.
    bool push(const uint64_t ins_addr, const uint64_t pred_addr)
    {
        bool castout = false;

        auto found = target.find(ins_addr);
        if ( found != target.end() ) {
            found->second = pred_addr;
            reorder(ins_addr);
        }
        else {
            if ( lru.size() >= max_entries ) {
                const uint64_t victim = lru.back();
                lru.pop_back();
                target.erase(target.find(victim));
                castout = true;
            }

            lru.push_front(ins_addr);
            target.insert(std::pair<uint64_t, uint64_t>(ins_addr, pred_addr));
        }

        return castout;
    }

    uint64_t predict(const uint64_t addr) const
    {
        auto found = target.find(addr);
        return (found == target.end()) ? 0 : found->second;
    }

    bool contains(const uint64_t addr) const { return target.find(addr) != target.end(); }

    void clear()
    {
        lru.clear();
        target.clear();
    }

private:
    void reorder(const uint64_t addr)
    {
        for ( auto itr = lru.begin(); itr != lru.end(); ++itr ) {
            if ( addr == (*itr) ) {
                lru.erase(itr);
                lru.push_front(addr);
                break;
            }
        }
    }

    const uint32_t                          max_entries;
    std::list<uint64_t>                     lru;
    std::unordered_map<uint64_t, uint64_t>  target;
};

} // namespace Vanadis
} // namespace SST

#endif
