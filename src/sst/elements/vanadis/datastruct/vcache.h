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

#ifndef _H_VANADIS_CACHE
#define _H_VANADIS_CACHE

#include <cstdint>
#include <list>
#include <type_traits>
#include <unordered_map>

namespace SST {
namespace Vanadis {

enum class VanadisCacheRecordDeletion {
    VANADIS_NO_DELETION,
    VANADIS_PERFORM_DELETE,
    VANADIS_PERFORM_DELETE_ARRAY
};

template <typename I, typename T, SST::Vanadis::VanadisCacheRecordDeletion D> class VanadisCache {
public:
    VanadisCache(const size_t cache_entries) : max_entries(cache_entries) { reset(); }

    ~VanadisCache() {
        clear();
    }

    void clear() {
        order_pos.clear();

        for (auto val_itr = data_values.begin(); val_itr != data_values.end(); val_itr++ ) {
            switch(D) {
                case SST::Vanadis::VanadisCacheRecordDeletion::VANADIS_PERFORM_DELETE:
                {
                    delete val_itr->second;
                } break;
                case SST::Vanadis::VanadisCacheRecordDeletion::VANADIS_PERFORM_DELETE_ARRAY:
                {
                    delete[] val_itr->second;
                } break;
                case SST::Vanadis::VanadisCacheRecordDeletion::VANADIS_NO_DELETION:
                {} break;
            }
        }

        ordering_q.clear();
        data_values.clear();
    }

    void reset() {
        clear();
        data_values.reserve(max_entries);
        order_pos.reserve(max_entries);
    }

    bool contains(const I& value) const { return (data_values.find(value) != data_values.end()); }

    T find(const I& key) {
        send_key_to_front(key);
        return data_values.find(key)->second;
    }

    void store(const I& key, T value) {
        if (LIKELY(contains(key))) {
            send_key_to_front(key);
	        data_values[key] = value;
        } else {
            kill_lru_key();
            data_values.insert(std::pair<I, T>(key, value));
            ordering_q.push_front(key);
            order_pos[key] = ordering_q.begin();
        }
    }

    void touch(const I& key) {
        if (LIKELY(contains(key))) {
            send_key_to_front(key);
        }
    }

    size_t size() const { return data_values.size(); }
    size_t capacity() const { return max_entries; }

private:
    void kill_lru_key() {
        // if we aren't full yet, then keep entries otherwise we will
        // throw away
        if (UNLIKELY(ordering_q.size() < max_entries)) {
            return;
        }

        const I remove_key = ordering_q.back();
        ordering_q.pop_back();
        order_pos.erase(remove_key);

        auto find_key = data_values.find(remove_key);

        switch(D) {
            case SST::Vanadis::VanadisCacheRecordDeletion::VANADIS_PERFORM_DELETE:
            {
                delete find_key->second;
            } break;
            case SST::Vanadis::VanadisCacheRecordDeletion::VANADIS_PERFORM_DELETE_ARRAY:
            {
                delete[] find_key->second;
            } break;
            case SST::Vanadis::VanadisCacheRecordDeletion::VANADIS_NO_DELETION:
            {} break;
        }

        data_values.erase(find_key);
    }

    // THE LRU ORDER IS THE SAME ORDER; FINDING A KEY IN IT IS NOT A SEARCH.
    //
    // This walked `ordering_q` linearly looking for the key, and the micro-op
    // cache this class holds has 1536 entries -- so every HIT, which is every
    // decode of a program that fits in the cache, walked up to 1536 list nodes.
    // At 14 % of the simulator it was the single most expensive symbol in the
    // profile once the issue stage stopped being it.
    //
    // A second map from key to its position in the list makes the move O(1),
    // and `std::list::splice` moves the node rather than destroying and
    // rebuilding it, so every OTHER key's iterator stays valid. The resulting
    // order is not merely equivalent to the old one, it is the same sequence of
    // keys -- which is what makes this a change with no behavioural content:
    // the same entry is evicted at the same moment.
    void send_key_to_front(const I& key) {
        auto pos = order_pos.find(key);

        if (LIKELY(pos != order_pos.end())) {
            ordering_q.splice(ordering_q.begin(), ordering_q, pos->second);
            pos->second = ordering_q.begin();
        }
    }

    const size_t max_entries;
    std::list<I> ordering_q;
    std::unordered_map<I, typename std::list<I>::iterator> order_pos;
    std::unordered_map<I, T> data_values;
};

} // namespace Vanadis
} // namespace SST

#endif
