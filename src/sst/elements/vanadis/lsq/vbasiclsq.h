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

#ifndef _H_VANADIS_BASIC_LSQ
#define _H_VANADIS_BASIC_LSQ

#include <sst/core/output.h>
#include <sst/core/subcomponent.h>
#include <sst/core/interfaces/stdMem.h>

#include "lsq/vlsq.h"
#include "lsq/vbasiclsqentry.h"
#include "lsq/vmemdeppredictor.h"
#include "util/vsignx.h"
#include "inst/vstorecond.h"

#include <algorithm>
#include <cassert>
#include <cinttypes>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>
#include <queue>

// A TRACE CALL THAT DOES NOT PAY FOR ITSELF WHEN NOBODY IS LISTENING.
//
// SST::Output::verbose is variadic, so the compiler cannot inline it however
// short its body is: every call reaches the out-of-line copy, and its va_start
// spills six integer and eight SSE registers to the stack BEFORE the level test
// that throws the message away. At verbosity 0 -- which is every run that is
// not being debugged -- that was 1.8 % of the whole simulator. One non-variadic
// call answers first. The do/while makes this a statement, so it substitutes
// for the call it replaces even as the unbraced body of an if or an else.
#ifndef VANADIS_VERB
#define VANADIS_VERB(obj, lvl, flg, ...)                                  \
    do {                                                                  \
        if ( (obj)->getVerboseLevel() >= (uint32_t)(lvl) ) {              \
            (obj)->verbose(CALL_INFO, lvl, flg, __VA_ARGS__);             \
        }                                                                 \
    } while ( 0 )
#endif

using namespace SST::Interfaces;

namespace SST {
namespace Vanadis {

// The first age a memory operation of a thread can be given. Zero is reserved
// to mean "no store": see next_age_ in the constructor.
#define VANADIS_LSQ_FIRST_AGE 1

// The stores ahead of one load, as much of them as the memory-dependence
// predictor is allowed to see: whether a named store instruction is one of
// them. Design B asks exactly this and nothing else, so the predictor never
// holds a pointer into the queue.
class VanadisBasicOlderStoreView : public VanadisStoreQView
{
public:
    VanadisBasicOlderStoreView(const std::deque<VanadisBasicStorePendingEntry*>& q, uint64_t load_age) :
        q_(q), load_age_(load_age)
    {}

    bool containsStorePC(uint64_t store_pc) const override
    {
        for ( auto* e : q_ ) {
            if ( e->getAge() >= load_age_ ) { break; }
            if ( e->getInstructionAddress() == store_pc ) { return true; }
        }
        return false;
    }

private:
    const std::deque<VanadisBasicStorePendingEntry*>& q_;
    const uint64_t                                    load_age_;
};

class VanadisBasicLoadStoreQueue : public SST::Vanadis::VanadisLoadStoreQueue
{
    public:
        SST_ELI_REGISTER_SUBCOMPONENT(VanadisBasicLoadStoreQueue, "vanadis", "VanadisBasicLoadStoreQueue",
                                            SST_ELI_ELEMENT_VERSION(1, 0, 0),
                                            "Implements a basic load-store queue with write buffer for use with the SST standardInterface",
                                            SST::Vanadis::VanadisLoadStoreQueue)

        SST_ELI_DOCUMENT_SUBCOMPONENT_SLOTS({ "memory_interface", "Set the interface to memory",
                                            "SST::Interfaces::StandardMem" })

        SST_ELI_DOCUMENT_PORTS({ "dcache_link", "Connects the LSQ to the data cache", {} })

        SST_ELI_DOCUMENT_PARAMS(
                { "max_stores", "Set the maximum number of stores permitted in the queue", "8" },
                { "max_loads", "Set the maximum number of loads permitted in the queue", "16" },
                { "address_mask", "Can mask off address bits if needed during construction of a operation", "0xFFFFFFFFFFFFFFFF"},
                { "issues_per_cycle", "Maximum number of issues the LSQ can attempt per cycle.", "2"},
                { "cache_line_width", "Number of bytes in a (L1) cache line", "64"},
                { "lsq_speculate", "1 lets a load issue past an older store whose address is not known yet, be answered from an older store's bytes, and be replayed when an older store later resolves onto bytes it has read. 0 is the strictly in-order queue, in which a load never passes a store.", "1"},
                { "mem_dep_speculation", "Alias of lsq_speculate, kept because the design document names it.", "1"},
                { "lsq_forward", "1 lets a load be answered out of an older store's bytes. 0 makes it wait for the store to reach memory, as the in-order queue does. Only meaningful when lsq_speculate is 1.", "1"},
                { "mem_dep_predictor", "Which memory-dependence predictor decides when a load waits: counter (a two-bit counter per load address), store_pc (the store that last made this load flush), none (never hold), hold (never speculate past a store whose address is unknown -- the control case)", "counter"},
                { "mdp_entries", "Two-bit counters in the counter predictor, rounded up to a power of two", "4096"},
                { "mdp_store_entries", "Entries in the store-address predictor, rounded up to a power of two", "4096"},
                { "mdp_counter_decay", "When the counter predictor counts down: retire, or speculated", "retire"}
            )

        SST_ELI_DOCUMENT_STATISTICS({ "bytes_read", "Count all the bytes read for data operations", "bytes", 1 },
                                    { "bytes_stored", "Count all the bytes written for data operations", "bytes", 1 },
                                    { "loads_issued", "Count the number of loads issued", "operations", 1 },
                                    { "stores_issued", "Count the number of stores issued", "operations", 1 },
                                    { "fences_issued", "Count the number of fences issued", "operations", 1},
                                    { "loads_executed", "Count the number of loads issued", "operations", 1 },
                                    { "stores_executed", "Count the number of stores issued", "operations", 1 },
                                    { "fences_executed", "Count the number of fences issued", "operations", 1},
                                    { "operations_pending", "Count the number of operations which are held by the LSQ and not ready to be issued to the memory subsystem", "operations", 1},
                                    { "loads_in_flight", "Count the number of loads which are in-flight", "operations", 1},
                                    { "stores_in_flight", "Count the number of stores which are in-flight", "operations", 1},
                                    { "store_buffer_entries", "Count the number of stores held in the store buffer", "operations", 1},
                                    { "split_stores", "Count the number of stores which are fractured due to cache boundaries", "operations", 1},
                                    { "split_loads", "Count the number of loads which are fractured due to cache boundaries", "operations", 1},
                                    { "addr_outside_space", "Count the accesses whose address does not fit address_mask and were therefore faulted rather than sent", "operations", 1},
                                    { "mem_loads_speculated", "Count the loads answered while at least one older store still had an unknown address", "operations", 1},
                                    { "mem_loads_forwarded", "Count the loads answered out of an older store's bytes, with no request sent to memory", "operations", 1},
                                    { "mem_violations", "Count the loads an older store resolved onto after they had already read", "operations", 1},
                                    { "mem_replays", "Count the flushes actually taken to re-execute such a load", "operations", 1},
                                    { "mem_predictor_holds", "Count the loads the memory-dependence predictor held back at least once", "operations", 1})


        VanadisBasicLoadStoreQueue(ComponentId_t id, Params& params, int coreid, int hwthreads) : VanadisLoadStoreQueue(id, params, coreid, hwthreads),
            max_stores(params.find<size_t>("max_stores", 8)),
        max_loads(params.find<size_t>("max_loads", 16)),
        max_issue_attempts_per_cycle(params.find("issues_per_cycle", 2))
        {
            std_mem_handlers = new VanadisBasicLoadStoreQueue::StandardMemHandlers(this, output);

            memInterface = loadUserSubComponent<Interfaces::StandardMem>(
                "memory_interface", ComponentInfo::SHARE_PORTS | ComponentInfo::INSERT_STATS, getTimeConverter("1ps"),
                new StandardMem::Handler2<SST::Vanadis::VanadisBasicLoadStoreQueue,&VanadisBasicLoadStoreQueue::processIncomingDataCacheEvent>(this));

            address_mask = params.find<uint64_t>("address_mask", 0xFFFFFFFFFFFFFFFFULL);

            cache_line_width = params.find<uint64_t>("cache_line_width", 64);

            op_q.resize(hw_threads);
            op_q_index = 0;
            op_q_size = 0;

            stores_pending.resize(hw_threads);
            stores_pending_index = 0;
            stores_pending_size = 0;

            stat_loads_issued = registerStatistic<uint64_t>("loads_issued", "1");
            stat_stores_issued = registerStatistic<uint64_t>("stores_issued", "1");
            stat_fences_issued = registerStatistic<uint64_t>("fences_issued", "1");

            stat_loads_executed = registerStatistic<uint64_t>("loads_executed", "1");
            stat_stores_executed = registerStatistic<uint64_t>("stores_executed", "1");
            stat_fences_executed = registerStatistic<uint64_t>("fences_executed", "1");

            stat_loaded_bytes = registerStatistic<uint64_t>("bytes_read", "1");
            stat_stored_bytes = registerStatistic<uint64_t>("bytes_stored", "1");

            stat_store_buffer_entries = registerStatistic<uint64_t>("store_buffer_entries", "1");
            stat_stores_pending = registerStatistic<uint64_t>("stores_in_flight", "1");
            stat_loads_pending = registerStatistic<uint64_t>("loads_in_flight", "1");
            stat_op_q_size = registerStatistic<uint64_t>("operations_pending");
            stat_addr_outside_space = registerStatistic<uint64_t>("addr_outside_space", "1");

            stat_mem_loads_speculated = registerStatistic<uint64_t>("mem_loads_speculated", "1");
            stat_mem_loads_forwarded  = registerStatistic<uint64_t>("mem_loads_forwarded", "1");
            stat_mem_violations       = registerStatistic<uint64_t>("mem_violations", "1");
            stat_mem_replays          = registerStatistic<uint64_t>("mem_replays", "1");
            stat_mem_predictor_holds  = registerStatistic<uint64_t>("mem_predictor_holds", "1");

            // SPECULATION IS THE DEFAULT, and turning it off restores the queue
            // this one grew out of: one in-order queue per hardware thread, a
            // load that never passes a store, and no violations to recover
            // from. Both names are read so that either may be used.
            spec_ = params.find<bool>("lsq_speculate", params.find<bool>("mem_dep_speculation", true));

            forward_ = params.find<bool>("lsq_forward", true);

            mdp_kind_ = params.find<std::string>("mem_dep_predictor", "counter");
            const size_t mdp_entries       = params.find<size_t>("mdp_entries", 4096);
            const size_t mdp_store_entries = params.find<size_t>("mdp_store_entries", 4096);
            const std::string decay        = params.find<std::string>("mdp_counter_decay", "retire");

            if ( (decay != "retire") && (decay != "speculated") ) {
                output->fatal(CALL_INFO, -1,
                    "Error: mdp_counter_decay is \"%s\"; it is either \"retire\" or \"speculated\".\n",
                    decay.c_str());
            }

            load_q.resize(hw_threads);
            ordered_ins_.resize(hw_threads, nullptr);
            // AGE 0 IS NOT AN AGE. A load records the age of the store it was
            // answered from, and 0 there means "answered by memory, from no
            // store at all". If a store could hold age 0 the two would be the
            // same number, and the violation check -- which asks whether the
            // resolving store is younger than whatever the load was answered
            // from -- would skip every load against the first store after a
            // queue clear. That is not a rare corner: the queues are cleared on
            // every branch mis-predict, so age 0 comes round constantly.
            next_age_.resize(hw_threads, VANADIS_LSQ_FIRST_AGE);
            waiting_loads_.resize(hw_threads, 0);
            forced_hold_pc_.resize(hw_threads, 0);
            forced_hold_armed_.resize(hw_threads, false);
            load_q_size = 0;

            // ONE TABLE PER HARDWARE THREAD. Sharing one would let a thread's
            // behaviour hold another thread's loads, and the table is a
            // kilobyte.
            mem_dep_.resize(hw_threads, nullptr);
            for ( int t = 0; t < hw_threads; ++t ) {
                mem_dep_[t] = vanadisMakeMemDepPredictor(
                    mdp_kind_, mdp_entries, mdp_store_entries, decay == "speculated");
                if ( nullptr == mem_dep_[t] ) {
                    output->fatal(CALL_INFO, -1,
                        "Error: mem_dep_predictor is \"%s\"; it is one of counter, store_pc, none.\n",
                        mdp_kind_.c_str());
                }
            }
        }


        virtual ~VanadisBasicLoadStoreQueue() {
            for (int i = 0; i < hw_threads; i++ ) {
                for(auto op_q_itr = op_q[i].begin(); op_q_itr != op_q[i].end(); ) {
                    delete (*op_q_itr);
                    op_q_itr = op_q[i].erase(op_q_itr);
                }
                for ( auto* entry : load_q[i] ) { delete entry; }
                load_q[i].clear();
                delete mem_dep_[i];
            }
            delete std_mem_handlers;
        }

        bool speculative() const override { return spec_; }

        // WHERE THE BACK-PRESSURE IS APPLIED. In the speculative queue a slot is
        // taken when the instruction is renamed, so these answer the core at
        // rename; in the in-order queue they answer it at issue, as before.
        bool storeFull() override { return spec_ ? (stores_pending_size >= max_stores) : (op_q_size >= max_stores); }
        bool loadFull() override { return spec_ ? (load_q_size >= max_loads) : (op_q_size >= max_loads); }
        bool storeBufferFull() override { return std_stores_in_flight.size() >= max_stores; }

        size_t storeSize() override { return spec_ ? stores_pending_size : op_q_size; }
        size_t loadSize() override { return spec_ ? load_q_size : op_q_size; }
        size_t storeBufferSize() override { return std_stores_in_flight.size(); }

        // A SLOT, TAKEN AT RENAME. Until a store holds a slot carrying an age
        // that says it is older, a younger load has no way to know it exists,
        // and the whole point of this queue is that a load can decide what to do
        // about a store whose address nobody has computed yet.
        void reserve(VanadisInstruction* ins) override
        {
            if ( !spec_ ) { return; }

            const uint32_t thr = ins->getHWThread();

            switch ( ins->getInstFuncType() ) {
            case INST_LOAD:
            {
                VanadisBasicLoadPendingEntry* entry =
                    new VanadisBasicLoadPendingEntry(ins->asLoad(), next_age_[thr]++);
                load_q[thr].push_back(entry);
                load_q_size++;
                reserved_loads_[ins] = entry;
                stat_loads_issued->addData(1);
            } break;
            case INST_STORE:
            {
                VanadisBasicStorePendingEntry* entry =
                    new VanadisBasicStorePendingEntry(ins->asStore(), next_age_[thr]++);
                stores_pending[thr].push_back(entry);
                stores_pending_size++;
                reserved_stores_[ins] = entry;
                stat_stores_issued->addData(1);
            } break;
            default:
                // A fence takes no slot. It needs no age either: the core will
                // not hand this queue anything younger than a fence it has not
                // executed, so every resolved operation in the queue when a
                // fence arrives is older than the fence.
                break;
            }
        }

        // A LOAD'S SLOT IS FREED WHEN THE LOAD RETIRES, not when its value
        // arrives: until it retires an older store may still resolve onto its
        // bytes. This is also where the counter predictor learns that a load
        // went through without costing anything.
        void commit(VanadisInstruction* ins) override
        {
            if ( !spec_ ) { return; }

            // Retiring is the last moment this pointer is safe to hold, so it is
            // dropped here whether or not orderedPending() has already noticed
            // that the instruction executed.
            if ( ordered_ins_[ins->getHWThread()] == ins ) { ordered_ins_[ins->getHWThread()] = nullptr; }

            if ( INST_LOAD != ins->getInstFuncType() ) { return; }

            const uint32_t thr = ins->getHWThread();
            const uint64_t pc  = ins->getInstructionAddress();

            for ( auto itr = load_q[thr].begin(); itr != load_q[thr].end(); ++itr ) {
                if ( (*itr)->getInstruction() != ins ) { continue; }

                VanadisBasicLoadPendingEntry* entry = *itr;

                // A LOAD THE SAFETY NET HELD PRODUCED NO EVIDENCE. It was not
                // allowed to speculate, so its retiring without a violation
                // says nothing about whether it would have violated, and
                // counting it down would leave a load that aliases every time
                // oscillating between "held" and "flushed" for ever.
                const bool was_forced = forced_hold_armed_[thr] && (forced_hold_pc_[thr] == pc);

                if ( !was_forced ) { mem_dep_[thr]->retiredClean(pc, entry->didSpeculate()); }
                else               { forced_hold_armed_[thr] = false; }

                if ( VanadisBasicLoadPendingEntry::WAITING == entry->getState() ) { waiting_loads_[thr]--; }
                dropFromLoadsPending(entry);
                reserved_loads_.erase(ins);
                load_q[thr].erase(itr);
                load_q_size--;
                delete entry;
                return;
            }
        }

        // THE FLUSH IS ABOUT TO BE TAKEN. Called while the record of what
        // happened is still here, because the repair that follows discards it.
        //
        // The second half is a safety net that no predictor provides and the
        // machine cannot run without. Recovery here is a whole-thread flush, so
        // the store that caused the violation is thrown away with the load and
        // re-executed from scratch -- and if nothing changed, it would race the
        // load exactly as before and flush again, for ever. Arming the load's
        // own address makes its very next execution wait for every older store,
        // which cannot violate; committing it disarms the net again.
        void noteReplay(VanadisInstruction* ins) override
        {
            if ( !spec_ ) { return; }

            const uint32_t thr = ins->getHWThread();
            const uint64_t pc  = ins->getInstructionAddress();

            for ( auto* entry : load_q[thr] ) {
                if ( entry->getInstruction() != ins ) { continue; }
                mem_dep_[thr]->violated(pc, entry->violatingStorePC());
                break;
            }

            forced_hold_pc_[thr]    = pc;
            forced_hold_armed_[thr] = true;
            stat_mem_replays->addData(1);
        }

        // The thread's address space has been replaced, so every load address
        // the table has learned about names something else now. This is the only
        // event that clears it: a table wiped on every branch mis-predict would
        // never hold anything.
        bool orderedPending(const uint32_t thread) override
        {
            if ( !spec_ ) { return false; }
            if ( nullptr == ordered_ins_[thread] ) { return false; }
            if ( ordered_ins_[thread]->completedExecution() ) {
                ordered_ins_[thread] = nullptr;
                return false;
            }
            return true;
        }

        void resetPredictors(const uint32_t thread) override
        {
            if ( !spec_ ) { return; }
            mem_dep_[thread]->reset();
            forced_hold_armed_[thread] = false;
        }

        void push(VanadisStoreInstruction* store_me) override
        {
            if ( spec_ ) { resolveStore(store_me); return; }

            op_q[store_me->getHWThread()].push_back( new VanadisBasicStoreEntry(store_me) );
            op_q_size++;
            stat_stores_issued->addData(1);
        }

        void push(VanadisLoadInstruction* load_me) override
        {
            if ( spec_ ) { resolveLoad(load_me); return; }

            op_q[load_me->getHWThread()].push_back( new VanadisBasicLoadEntry(load_me) );
            op_q_size++;
            stat_loads_issued->addData(1);
        }

        void push(VanadisFenceInstruction* fence) override
        {
            // Nothing younger reaches this queue until it has executed.
            if ( spec_ ) { ordered_ins_[fence->getHWThread()] = fence; }

            op_q[fence->getHWThread()].push_back( new VanadisBasicFenceEntry(fence) );
            op_q_size++;
            stat_fences_issued->addData(1);
        }

        void clearLSQByThreadID(const uint32_t thread) override
        {
            // Iterate over the queue, anything with a matching thread ID is
            // first deleted and then removed from the queue, otherwise entry
            // is left alone

            op_q_size -= op_q[thread].size();
            for(auto op_q_itr = op_q[thread].begin(); op_q_itr != op_q[thread].end(); ) {
                delete (*op_q_itr);
                op_q_itr = op_q[thread].erase(op_q_itr);

            }

            if ( spec_ ) {
                // In this mode the in-flight list holds pointers it does not
                // own: the load queue does. Take the pointers out of it first,
                // then free the queue.
                for(auto load_itr = loads_pending.begin(); load_itr != loads_pending.end(); ) {
                    if( (*load_itr)->getHWThread() == thread ) {
                        load_itr = loads_pending.erase(load_itr);
                    } else {
                        ++load_itr;
                    }
                }

                for ( auto* entry : load_q[thread] ) {
                    reserved_loads_.erase(entry->getInstruction());
                    delete entry;
                }
                load_q_size -= load_q[thread].size();
                load_q[thread].clear();
                waiting_loads_[thread] = 0;

                for ( auto* entry : stores_pending[thread] ) {
                    reserved_stores_.erase(entry->getInstruction());
                }

                // Every entry that could have been compared against is gone, so
                // the ages may start again. The predictors are NOT cleared: what
                // they have learned about the program survives a repair, and a
                // table emptied on every mis-predict would stay empty.
                next_age_[thread] = VANADIS_LSQ_FIRST_AGE;
                ordered_ins_[thread] = nullptr;
            } else {
                for(auto load_itr = loads_pending.begin(); load_itr != loads_pending.end(); ) {
                    if( (*load_itr)->getHWThread() == thread ) {
                        delete (*load_itr);
                        load_itr = loads_pending.erase(load_itr);
                    } else {
                        ++load_itr;
                    }
                }
            }

            stores_pending_size -= stores_pending[thread].size();
            for(auto store_itr = stores_pending[thread].begin(); store_itr != stores_pending[thread].end(); ) {
                delete (*store_itr);
                store_itr = stores_pending[thread].erase(store_itr);
            }
        }

        // must be implemented to allow the memory system to initialize itself during
        // boot-up
        void init(unsigned int phase) override
        {
            memInterface->init(phase);

            // update the cache line size each cycle to make sure we get updates
            cache_line_width = memInterface->getLineSize();

            VANADIS_VERB(output, 2, 0, "updating cache line size to: %" PRIu64 "\n", cache_line_width);
        }

        void printStatus(SST::Output& out) override
        {
            int32_t next_line = 0;

            if(output->getVerboseLevel() >= 16) {
                for (int i = 0; i < hw_threads; i++) {
                    for(auto op_q_itr = op_q[i].begin(); op_q_itr != op_q[i].end(); op_q_itr++) {
                        VanadisBasicLoadStoreEntryOp op_type = (*op_q_itr)->getEntryOp();

                        out.verbose(CALL_INFO, 16, VANADIS_DBG_LSQ_LOAD_FLG, "-> [%4" PRId32 "] type: %5s ins: 0x%8" PRI_ADDR " thr: %4" PRIu32 "\n",
                            next_line,
                            (op_type == VanadisBasicLoadStoreEntryOp::LOAD) ? "LOAD" :
                            (op_type == VanadisBasicLoadStoreEntryOp::STORE) ? "STORE" : "FENCE",
                            (*op_q_itr)->getInstruction()->getInstructionAddress(),
                            (*op_q_itr)->getInstruction()->getHWThread());
                        next_line++;
                    }
                }
            }
        }

        void tick(uint64_t cycle) override
        {
            if(output->getVerboseLevel() >= 16) {
                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "-> tick LSQ at cycle %" PRIu64 "\n", cycle);

                if(loads_pending.size() > 0) {
                    for(int i = loads_pending.size() - 1; i >= 0; i--) {
                        VanadisBasicLoadPendingEntry* load_entry = loads_pending.at(i);
                        VANADIS_VERB(output, 8, 0, "-->   load[%5d] ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " / addr: 0x%" PRI_ADDR " / width: %" PRIu64 "\n",
                            i, load_entry->getLoadInstruction()->getInstructionAddress(),
                            load_entry->getLoadInstruction()->getHWThread(),
                            load_entry->getLoadAddress(), load_entry->getLoadWidth());
                    }
                }

                if(stores_pending_size > 0) {
                    for (int t = 0; t < hw_threads; t++) {
                        for(int i = stores_pending[t].size() - 1; i >= 0; i--) {
                            VanadisBasicStorePendingEntry* store_entry = stores_pending[t].at(i);

                            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> stores[%5d] ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " / addr: 0x%" PRI_ADDR " / width: %" PRIu64 "\n",
                                i, store_entry->getStoreInstruction()->getInstructionAddress(),
                                store_entry->getStoreInstruction()->getHWThread(),
                                store_entry->getStoreAddress(), store_entry->getStoreWidth());
                        }
                    }
                }
            }

            stat_op_q_size->addData(spec_ ? (load_q_size + stores_pending_size) : op_q_size);
            stat_loads_pending->addData(loads_pending.size());
            stat_stores_pending->addData(std_stores_in_flight.size());
            stat_store_buffer_entries->addData(stores_pending_size);

            // this can be called multiple times per cycle
            for(uint32_t attempt = 0; attempt < max_issue_attempts_per_cycle; ++attempt) {
                if ( spec_ ) {
                    attempt_to_issue_speculative(op_q_index);
                    op_q_index = (op_q_index + 1) % hw_threads;
                    continue;
                }
                if (op_q_size == 0)
                    break;
                const bool attempt_result = attempt_to_issue(cycle, attempt, op_q_index);
                op_q_index = (op_q_index + 1) % hw_threads;
            }

            // attempt to issue any front of ROB stores into memory system
            for (int i = 0; i < hw_threads; i++) {
                bool issued = issueStoreFront(stores_pending_index);
                stores_pending_index = (stores_pending_index + 1) % hw_threads;
                if (issued) break; // one per cycle TODO: parameterize
            }
        }

    protected:

        class StandardMemHandlers : public Interfaces::StandardMem::RequestHandler
        {
            public:
                friend class VanadisBasicLoadStoreQueue;

                StandardMemHandlers(VanadisBasicLoadStoreQueue* lsq, SST::Output* output) :
                        Interfaces::StandardMem::RequestHandler(output), lsq(lsq) {}

                virtual ~StandardMemHandlers() {}


                virtual void copyLoadResp(VanadisLoadInstruction* load_ins, uint16_t* target_reg,
                    uint16_t* target_isa_reg,
                    uint32_t* target_thread,VanadisBasicLoadPendingEntry* load_entry,
                    uint8_t fp)
                {

                    *target_thread = load_ins->getHWThread();
                    VANADIS_VERB(out, 16, VANADIS_DBG_LSQ_LOAD_FLG, " (ScalarLSQ) -> copyLoadResp thr=%d\n", *target_thread);
                    if(fp==true)
                    {
                        *target_isa_reg =  load_ins->getISAFPRegOut(0);
                        *target_reg = load_ins->getPhysFPRegOut(0);
                    }
                    else
                    {
                        *target_isa_reg =  load_ins->getISAIntRegOut(0);
                        *target_reg = load_ins->getPhysIntRegOut(0);
                    }
                }

                virtual void processLLSC(StandardMem::WriteResp* ev, VanadisStoreInstruction* store_ins,VanadisBasicStorePendingEntry* store_entry)
                {
                    VANADIS_VERB(out, 16, VANADIS_DBG_LSQ_STORE_FLG, " (ScalarLSQ) -> processLLSC\n");
                    const uint16_t value_reg = store_ins->getPhysIntRegOut(0);

                    VanadisStoreConditionalInstruction* store_cond_ins = store_ins->asStoreConditional();

                    if(UNLIKELY(nullptr == store_cond_ins)) {
                        out->fatal(CALL_INFO, -1, "Unable to cast an LLSC_STORE into a store-conditional, logic failure.\n");
                    }

                    if (ev->getSuccess()) {
                        const int64_t success_result = store_cond_ins->getResultSuccess();

                        VANADIS_VERB(out, 9, VANADIS_DBG_LSQ_STORE_FLG,
                                        "---> LSQ LLSC-STORE rt: %" PRIu64 " <- %" PRIu16 " (success)\n",
                                        success_result, value_reg);
                        lsq->registerFiles->at(store_ins->getHWThread())->setIntReg<int64_t>(value_reg,
                            success_result);
                    } else {
                        const int64_t failure_result = store_cond_ins->getResultFailure();

                        VANADIS_VERB(out, 9, VANADIS_DBG_LSQ_STORE_FLG, "---> LSQ LLSC-STORE rt: %" PRIu64 " <- %" PRIu16 " (failed)\n",
                                        failure_result, value_reg);
                        lsq->registerFiles->at(store_ins->getHWThread())->setIntReg<uint64_t>(value_reg,
                            failure_result);
                    }
                }

                virtual void handle(StandardMem::ReadResp* ev)
                {
                    VANADIS_VERB(out, 16, VANADIS_DBG_LSQ_LOAD_FLG, "-> handle read-response (virt-addr: 0x%" PRI_ADDR ")\n", ev->vAddr);
                    lsq->stat_loaded_bytes->addData(ev->size);

                    auto load_itr = lsq->loads_pending.begin();
                    VanadisBasicLoadPendingEntry* load_entry = nullptr;

                    for(; load_itr != lsq->loads_pending.end(); load_itr++) {
                        if((*load_itr)->containsRequest(ev->getID())) {
                            load_entry = *load_itr;
                            break;
                        }
                    }
                    VanadisLoadInstruction* load_ins = nullptr;
                    if ( load_entry ) {
                        load_ins = load_entry->getLoadInstruction();
                    }
                    #ifdef VANADIS_BUILD_DEBUG
                    if ( lsq->isDbgAddr( ev->vAddr ) ) {
                    printf("ReadResp::%s() load_address=%#" PRIx64 " %s ins_addr=%#" PRIx64 "\n",__func__,
                        ev->vAddr, ev->getFail()? "Failed":"Success", load_ins ? load_ins->getInstructionAddress() : 0 );
                    }
                    #endif

                    if(nullptr == load_entry) {
                        // not found, so previous cleared by a branch mis-predict ignore
                        return;
                    }

                    if(out->getVerboseLevel() >= 16) {
                        VANADIS_VERB(out, 16, VANADIS_DBG_LSQ_LOAD_FLG,
                                        "--> LSQ match load entry, unpacking payload "
                                        "(load-addr: 0x%0" PRI_ADDR ", load-thr: %" PRIu32 ").\n",
                                        ev->pAddr, load_entry->getHWThread());
                    }

                    const uint16_t load_width = ev->size;
                    const uint64_t load_address = load_entry->getLoadAddress();


                    if ( ev->getFail())
                    {
                        VANADIS_VERB(out, 16, VANADIS_DBG_LSQ_LOAD_FLG,
                                        "--> ev failed "
                                        "(load-addr: 0x%0" PRI_ADDR ", load-thr: %" PRIu32 ").\n",
                                        ev->pAddr, load_entry->getHWThread());
                        load_ins->flagError();
                    }
                    if(ev->vAddr < 64)
                    {
                        VANADIS_VERB(out, 16, VANADIS_DBG_LSQ_LOAD_FLG,
                                        "ev virtual addr<64 "
                                        "(load-addr: 0x%0" PRI_ADDR ",virtual-addr: 0x%0" PRI_ADDR ", load-thr: %" PRIu32 ").\n",
                                        ev->pAddr, ev->vAddr, load_entry->getHWThread());
                        load_ins->flagError();
                    }

                    uint16_t target_reg = 0;
                    uint16_t target_isa_reg = 64;
                    uint32_t target_thread     = 0;
                    uint8_t fp = 0;
                    uint64_t reg_offset  = load_ins->getRegisterOffset();
                    uint64_t addr_offset = ev->vAddr - load_address;
                    uint32_t reg_width = 0;

                    // The level this builds its string for is the level the
                    // verbose() call below prints at. It used to read `>= 0`,
                    // which is a tautology on an unsigned level: every load
                    // response, at every verbosity, hex-formatted its whole
                    // payload into an ostringstream and copied it out with
                    // str() to build a string verbose() then discarded.
                    if(out->getVerboseLevel() >= 16) {
                        std::ostringstream str;
                        str << ", Payload: 0x";
                        str << std::hex << std::setfill('0');
                        for ( std::vector<uint8_t>::iterator it = ev->data.begin(); it != ev->data.end(); it++ ) {
                            str << std::setw(2) << static_cast<unsigned>(*it);
                        }
                        VANADIS_VERB(out, 16, VANADIS_DBG_LSQ_LOAD_FLG, "---> LSQ recv load event ins: 0x%" PRI_ADDR " / hw-thr: %" PRIu32 " / entry-addr: 0x%" PRI_ADDR " / entry-width: %" PRIu16 " / reg-offset: %" PRIu64 " / ev-addr: 0x%" PRI_ADDR " / ev-width: %" PRIu64 " / addr-offset %" PRIu64 " / sign-extend: %s / target-isa-reg: %" PRIu16 " / target-phys-reg: %" PRIu16 " / reg-type: %s / %s\n",
                            load_ins->getInstructionAddress(), load_entry->getHWThread(), load_address, load_width, reg_offset, ev->vAddr, ev->size,
                            addr_offset, (load_ins->performSignExtension() ? "yes" : "no"),
                            target_isa_reg, target_reg,
                            (load_ins->getValueRegisterType() == LOAD_INT_REGISTER) ? "int" : "fp",str.str().c_str());

                    }


                    switch(load_ins->getValueRegisterType()) {
                    case LOAD_INT_REGISTER: {

                        if ( ! load_ins->trapsError() ) {

                            copyLoadResp(load_ins, &target_reg,&target_isa_reg,&target_thread,load_entry,fp);

                            assert(target_isa_reg < load_ins->getISAOptions()->countISAIntRegisters());
                            //sst_assert(target_isa_reg < load_ins->getISAOptions()->countISAIntRegisters(),
                            //    CALL_INFO, -1, "target_isa_reg is incorrect");

                            if(target_reg != load_ins->getISAOptions()->getRegisterIgnoreWrites()) {
                                reg_width = lsq->registerFiles->at(target_thread)->getIntRegWidth();
                                std::vector<uint8_t> register_value(reg_width);
                                // copy entire register here
                                lsq->registerFiles->at(target_thread)->copyFromIntRegister(target_reg, 0, &register_value[0], reg_width);

                                // A LOAD RESPONSE THAT DOES NOT FIT THE REGISTER IT IS FOR.
                                //
                                // It was `assert(...)`, which says nothing: the
                                // abort names this line and no address, and the
                                // three quantities that decide it are all in
                                // registers the core has. Every one of them is a
                                // clue -- an `addr_offset` of 2^64-something is a
                                // response whose address is not the one the entry
                                // was opened for, a large `ev->size` is a response
                                // for another request, and the instruction address
                                // says which load. Printing them turns a run that
                                // dies with a line number into one that says what
                                // it was doing, which is the difference between a
                                // day and ten minutes.
                                if(UNLIKELY((reg_offset + addr_offset + ev->size) > reg_width)) {
                                    out->fatal(CALL_INFO, -1,
                                        "load response does not fit the register: ins 0x%" PRI_ADDR " (%s), "
                                        "load-addr 0x%" PRI_ADDR " width %" PRIu16 ", reg-offset %" PRIu64 ", "
                                        "response vAddr 0x%" PRI_ADDR " pAddr 0x%" PRI_ADDR " size %" PRIu64 ", "
                                        "addr-offset %" PRIu64 ", reg-width %" PRIu32 ", requests-left %d\n",
                                        load_ins->getInstructionAddress(), load_ins->getInstCode(),
                                        load_address, load_width, reg_offset,
                                        ev->vAddr, ev->pAddr, (uint64_t) ev->size, addr_offset, reg_width,
                                        (int) load_entry->countRequests());
                                }

                                for(auto i = 0; i < ev->size; ++i) {
                                    register_value.at(reg_offset + addr_offset + i) = ev->data[i];
                                }

                                // if we are the last request to be processed for this load (if any were split)
                                // and we promised to do sign extension, then perform it now
                                if(load_entry->countRequests() == 1) {
                                    if(load_ins->performSignExtension()) {
                                        if((register_value.at(reg_offset + addr_offset + load_width - 1) & 0x80) != 0) {
                                            for(auto i = reg_offset + addr_offset + load_width; i < reg_width; ++i) {
                                                register_value.at(i) = 0xFF;
                                            }
                                        } else {
                                            for(auto i = reg_offset + addr_offset + load_width; i < reg_width; ++i) {
                                                register_value.at(i) = 0x00;
                                            }
                                        }
                                    } else {
                                        for(auto i = reg_offset + addr_offset + load_width; i < reg_width; ++i) {
                                            register_value.at(i) = 0x00;
                                        }
                                    }
                                }

                                lsq->registerFiles->at(target_thread)->copyToIntRegister(target_reg, 0, &register_value[0], register_value.size());
                            }
                        }
                    } break;
                    case LOAD_FP_REGISTER: {

                        if ( ! load_ins->trapsError() ) {
                        fp=1;
                        copyLoadResp(load_ins, &target_reg,&target_isa_reg,&target_thread,load_entry,fp);


                        reg_width = lsq->registerFiles->at(target_thread)->getFPRegWidth();
                        std::vector<uint8_t> register_value(reg_width);

                        // copy entire register here
                        lsq->registerFiles->at(target_thread)->copyFromFPRegister(target_reg, 0, &register_value[0], reg_width);

                        assert((reg_offset + addr_offset + ev->size) <= reg_width);

                        for(auto i = reg_offset + addr_offset; i < ev->size; ++i) {
                            register_value.at(reg_offset + addr_offset + i) = ev->data[i];
                        }

                        if(load_entry->countRequests() == 1) {
                            for(auto i = reg_offset + addr_offset + load_width; i < reg_width; ++i) {
                                register_value.at(i) = 0xff;
                            }
                        }

                        lsq->registerFiles->at(target_thread)->copyToFPRegister(target_reg, 0, &register_value[0], reg_width);
                        }
                    } break;
                    default:
                        out->fatal(CALL_INFO, -1, "Unknown register type.\n");
                    }

                    ///////////////////////////////////////////////////////////////////////////////////

                    load_entry->removeRequest(ev->getID());

                    if(0 == load_entry->countRequests()) {
                        if(out->getVerboseLevel() >= 9) {
                            VANADIS_VERB(out, 9, VANADIS_DBG_LSQ_LOAD_FLG,
                                "---> LSQ Execute: %s (0x%" PRI_ADDR " / thr: %" PRIu32 ") load data instruction "
                                "marked executed.\n",
                                load_ins->getInstCode(), load_ins->getInstructionAddress(), load_ins->getHWThread());
                        }

                        load_ins->markExecuted();
                        lsq->stat_loads_executed->addData(1);
                        lsq->loads_pending.erase(load_itr);

                        // In the speculative queue the load queue owns this
                        // entry and keeps it until the load retires: an older
                        // store may still resolve onto the bytes it has just
                        // read, and the record of what it read is what says so.
                        if ( lsq->spec_ ) {
                            load_entry->setState(VanadisBasicLoadPendingEntry::DONE);
                        } else {
                            delete load_entry;
                        }
                    } else {
                        if(out->getVerboseLevel() >= 9) {
                            VANADIS_VERB(out, 9, VANADIS_DBG_LSQ_LOAD_FLG,
                                "---> LSQ Execute: %s (0x%" PRI_ADDR " / thr:%" PRIu32 ") does not have all requests completed yet %zu left, will not execute until all done.\n",
                                    load_ins->getInstCode(), load_ins->getInstructionAddress(), load_ins->getHWThread(), load_entry->countRequests());
                        }
                    }
                    delete ev;
                }

                virtual void handle(StandardMem::WriteResp* ev)
                {
                    VANADIS_VERB(out, 9, VANADIS_DBG_LSQ_STORE_FLG, "-> handle write-response (virt-addr: 0x%" PRI_ADDR ")\n", ev->vAddr);
                    lsq->stat_stored_bytes->addData(ev->size);

                    bool std_store_found = false;


                    auto iter = lsq->std_stores_in_flight.find( ev->getID() );
                    if ( iter != lsq->std_stores_in_flight.end() ) {
                        lsq->std_stores_in_flight.erase(iter);
                        std_store_found = true;
                    }

                    if(std_store_found) {
                        VANADIS_VERB(out, 9, VANADIS_DBG_LSQ_STORE_FLG, "--> write-response is a standard store is matched and cleared from in-flight operations successfully.\n");
                        delete ev;
                        return;
                    }

                    // this was not a standard store OR was removed by a branch mis-predict but we need to find
                    // out now
                    int thr = ev->tid;
                    if(0 == lsq->stores_pending[thr].size()) {
                        // no other pending stores so this request is free and can move on
                        delete ev;
                        return;
                    }

                    VanadisBasicStorePendingEntry* store_entry = lsq->stores_pending[thr].front();

                    if(store_entry->containsRequest(ev->getID())) {
                        VanadisStoreInstruction* store_ins = store_entry->getStoreInstruction();

                        switch(store_ins->getTransactionType())
                        {
                            case MEM_TRANSACTION_LLSC_STORE:
                            {
                                processLLSC(ev,store_ins,store_entry);

                                store_ins->markExecuted();
                                lsq->stores_pending[thr].erase(lsq->stores_pending[thr].begin());
                                lsq->stores_pending_size--;
                                delete store_entry;
                                delete ev;
                            } break;
                            case MEM_TRANSACTION_LOCK:
                            {
                                store_ins->markExecuted();
                                lsq->stores_pending[thr].erase(lsq->stores_pending[thr].begin());
                                lsq->stores_pending_size--;
                                delete store_entry;
                                delete ev;
                            } break;
                            default:
                            {
                                // this is a logical error. fatal()
                                out->fatal(CALL_INFO, -1, "Error - reached a transaction NONE or LLSC_LOAD in a store return. Logical error (ins: 0x%" PRI_ADDR " / thr: %" PRIu32 ")\n",
                                    store_ins->getInstructionAddress(), store_ins->getHWThread());
                            } break;
                        }
                    } else {
                        delete ev;
                        return;
                    }
                }

                VanadisBasicLoadStoreQueue* lsq;
        };

        void processIncomingDataCacheEvent(StandardMem::Request* ev)
        {
            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "received incoming data cache request -> processIncomingDataCacheEvent()\n");

            assert(ev != nullptr);
            assert(std_mem_handlers != nullptr);

            ev->handle(std_mem_handlers);
            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "completed pass off to incoming handlers\n");
        }

        bool issueStoreFront(uint32_t thr)
        {
            if(stores_pending[thr].empty()) {
                return false;
            }

            if(output->getVerboseLevel() >= 16) {
                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "issue store-front (thr: %" PRIu32 ")-> check store is front of ROB and attempt to issue\n", thr);
            }

            // check the front store of the pending queue, if this isn't currently dispatched
            // and is front of ROB, then we can execute it into the memory system
            VanadisBasicStorePendingEntry* current_store = stores_pending[thr].front();

            // A STORE WHOSE ADDRESS IS NOT KNOWN YET. Its slot was taken when it
            // was renamed, so it can be sitting at the front of the store queue
            // -- and even at the head of the reorder buffer -- before the
            // register its address is computed from has been produced. There is
            // nothing to send until it resolves.
            if( UNLIKELY(! current_store->isResolved()) ) {
                return false;
            }

            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "-> current pending store\n");
            VanadisStoreInstruction* store_ins = current_store->getStoreInstruction();
            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "-> current store\n");
            if(output->getVerboseLevel() >= 16)
            {
                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "-> current store queue front is ins addr: 0x%" PRI_ADDR "\n", store_ins->getInstructionAddress());
            }

            // check we have not already dispatched this entry
            if( UNLIKELY(!current_store->isDispatched()) )
            {
                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "--> store-front is not already dispatched so attempt to put into memory system.\n");

                if( UNLIKELY(store_ins->checkFrontOfROB()) && LIKELY(store_ins->completedIssue()) )
                {
                    VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "---> store is at front of ROB so OK to push into memory system.\n");

                    // store instruction is current front of ROB so ready to be send to memory system
                    bool issue_result = issueStore(current_store, store_ins);

                    VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "---> attempt to issue store result is: %s\n",
                        issue_result ? "success" : "failed");

                    // this was a standard store (not LLSC/LOCK) and we issued into system successfully
                    if(LIKELY(issue_result))
                    {
                        stores_pending[thr].pop_front();
                        stores_pending_size--;


                        if(output->getVerboseLevel() >= 16) {
                            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "---> issued store: 0x%" PRI_ADDR " / hw_thr: %" PRIu32 " / sw_thr: %" PRIu32 " into memory system using standard store operation\n",
                                store_ins->getInstructionAddress(), store_ins->getHWThread(),current_store->getSWThr());
                        }
                        delete current_store;

                        // mark executed
                        store_ins->markExecuted();
                        VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "---> issued store: 0x%" PRI_ADDR " / hw_thr: %" PRIu32 " / sw_thr: %" PRIu32 " / numStores: %" PRIu32 "\n",
                                store_ins->getInstructionAddress(), store_ins->getHWThread(),current_store->getSWThr(), store_ins->getNumStores());
                        stat_stores_executed->addData(1);
                    }
                    else
                    {
                        VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "---> issued non-standard store: 0x%" PRI_ADDR " / thr: %" PRIu32 " (marked dispatch, will stall until response)\n",
                            store_ins->getInstructionAddress(), store_ins->getHWThread());
                        current_store->markDispatched();
                    }
                    return issue_result;
                } else {
                    VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "----> store is not at ROB front so need to wait until this is marked\n");
                }
            } else {

                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "-> current store queue front is already dispatched. Returning\n");
            }
            return false;
        }

        virtual void getStoreTarget(VanadisBasicStorePendingEntry* store_entry,
            VanadisStoreInstruction* store_ins, uint16_t* target_thread, uint16_t* reg)
        {
            uint16_t thr = store_entry->getHWThread();
            uint16_t regtmp = (store_ins->getValueRegisterType() == STORE_FP_REGISTER) ? store_ins->getPhysFPRegIn(0) : store_ins->getPhysIntRegIn(1);
            *target_thread = thr;
            *reg = regtmp;

        }

        bool issueStore(VanadisBasicStorePendingEntry* store_entry,
            VanadisStoreInstruction* store_ins)
        {

            const uint64_t store_address = store_entry->getStoreAddress();
            const uint64_t store_width   = store_entry->getStoreWidth();
            StandardMem::Request* store_req = nullptr;
            std::vector<uint8_t> payload(store_width);
            uint16_t target_thread;
            uint16_t target_reg;

            #ifdef VANADIS_BUILD_DEBUG
            if ( isDbgInsAddr( store_ins->getInstructionAddress() ) || isDbgAddr( store_address ) ) {
                printf("%s() ins_addr=%#" PRIx64 " store_address=%#" PRIx64 "\n",__func__,store_ins->getInstructionAddress(), store_address);
            }
            #endif

            // The same refusal on the store side, and here it is never
            // speculative: a store issues only from the head of the reorder
            // buffer, so an address outside the space is one the program really
            // computes. Refusing to send it is what keeps a masked address from
            // being written to -- the load side dies loudly on a masked address
            // and the store side would not, it would write the program's bytes
            // somewhere else and carry on.
            if( UNLIKELY(! addressFitsSpace(store_address, store_width)) ) {
                noteAddressOutsideSpace("store", store_ins->getInstructionAddress(), store_address, store_width);
                store_ins->flagError();
                // Answering false is what the caller reads as "not issued": it
                // marks the entry dispatched itself, so the queue does not try
                // this store again while the core stops on it.
                return false;
            }

            // THE WORK COUNTER. This store is committed -- it issues only from
            // the head of the reorder buffer -- so if it touches the counter the
            // program keeps its own work in, its bytes are the counter's new
            // architectural value and are published to the core here. Only a
            // plain store is counted: a store-conditional may fail, and a
            // counter written with one would be published before the machine
            // knew whether it had happened.
            if( UNLIKELY(storeTouchesWork(store_address, store_width))
                && (store_ins->getTransactionType() == MEM_TRANSACTION_NONE) ) {
                std::vector<uint8_t> seen(store_width);
                uint16_t work_thread, work_reg;
                getStoreTarget(store_entry, store_ins, &work_thread, &work_reg);
                registerFiles->at(work_thread)->copyFromRegister(
                    work_reg, store_ins->getRegisterOffset(), &seen[0], store_width,
                    store_ins->getValueRegisterType() == STORE_FP_REGISTER);
                noteWorkStore(store_address, &seen[0], store_width);
            }

            const bool needs_split = operationStraddlesCacheLine(store_address, store_width);
            if(output->getVerboseLevel() >= 8)
            {
                std::vector<uint8_t> tmp(store_width);
                getStoreTarget(store_entry,store_ins, &target_thread, &target_reg);
                registerFiles->at(target_thread)->copyFromRegister(target_reg, store_ins->getRegisterOffset(), &tmp[0], store_width,
                    store_ins->getValueRegisterType() == STORE_FP_REGISTER);
                std::ostringstream str;
                str << ", Payload: 0x";
                str << std::hex << std::setfill('0');
                for ( std::vector<uint8_t>::iterator it = tmp.begin(); it != tmp.end(); it++ ) {
                    str << std::setw(2) << static_cast<unsigned>(*it);
                }
                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "--> thr %d, issue-store at ins: 0x%" PRI_ADDR " / store-addr: 0x%" PRI_ADDR " / width: %" PRIu64 " / partial: %s / split: %s / offset: %" PRIu32 " / %s\n",
                    store_ins->getHWThread(), store_ins->getInstructionAddress(), store_address, store_width, store_ins->isPartialStore() ? "yes" : "no", needs_split ? "yes" : "no",
                    store_ins->getRegisterOffset(), str.str().c_str());
            }

            // if the store is not a split operation, then copy payload we are good to go, if it is split
            // handle this case later after we do a load of address and width calculation
            if(LIKELY(! needs_split)) {
                    getStoreTarget(store_entry,store_ins, &target_thread, &target_reg);
                    registerFiles->at(target_thread)->copyFromRegister(target_reg, store_ins->getRegisterOffset(), &payload[0], store_width,
                store_ins->getValueRegisterType() == STORE_FP_REGISTER);
            }

            switch(store_ins->getTransactionType()) {
            case MEM_TRANSACTION_NONE:
            {
                if(UNLIKELY(needs_split)) {
                    if(output->getVerboseLevel() >= 9) {
                        VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_STORE_FLG, "---> [memory-transaction]: standard split-store\n");
                    }

                    const uint64_t store_width_right = (store_address + store_width) % cache_line_width;
                    const uint64_t store_width_left  = store_width - store_width_right;

                    assert(store_width_left > 0);
                    assert(store_width_right > 0);

                    const uint64_t store_address_right = store_address + store_width_left;

                    if(output->getVerboseLevel() >= 9) {
                        VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_STORE_FLG, "---> store-left-at: 0x%" PRI_ADDR " left-width: %" PRIu64 ", store-right-at: 0x%" PRI_ADDR " right-width: %" PRIu64 "\n",
                            store_address, store_width_left, store_address_right, store_width_right);
                    }

                    payload.resize(store_width_left);
                    getStoreTarget(store_entry,store_ins, &target_thread, &target_reg);
                    registerFiles->at(target_thread)->copyFromRegister(target_reg, store_ins->getRegisterOffset(), &payload[0], store_width_left,
                    store_ins->getValueRegisterType() == STORE_FP_REGISTER);

                    store_req = new StandardMem::Write(store_address & address_mask, payload.size(), payload,
                        false, 0, store_address, store_ins->getInstructionAddress(), store_ins->getHWThread());

                    std_stores_in_flight.insert(store_req->getID());
                    memInterface->send(store_req);

                    payload.clear();

                    payload.resize(store_width_right);


                    getStoreTarget(store_entry,store_ins, &target_thread, &target_reg);
                    registerFiles->at(target_thread)->copyFromRegister(target_reg, store_ins->getRegisterOffset()+store_width_left, &payload[0], store_width_right,
                    store_ins->getValueRegisterType() == STORE_FP_REGISTER);

                    store_req = new StandardMem::Write(store_address_right & address_mask, payload.size(), payload,
                        false, 0, store_address_right, store_ins->getInstructionAddress(), store_ins->getHWThread());
                    memInterface->send(store_req);
                    std_stores_in_flight.insert(store_req->getID());

                    return true;
                } else {
                    if(output->getVerboseLevel() >= 9) {
                        VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_STORE_FLG, "---> [memory-transaction]: standard store ins: 0x%" PRI_ADDR " store-at: 0x%" PRI_ADDR " width: %" PRIu64 "\n",
                            store_ins->getInstructionAddress(), store_address, store_width);

                        VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_STORE_FLG, "-----> payload = {");

                        for(auto i = 0; i < payload.size(); ++i) {
                            VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_STORE_FLG, " %x", payload[i]);
                        }

                        VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_STORE_FLG, "}\n");
                    }

                    store_req = new StandardMem::Write(store_address & address_mask, payload.size(), payload,
                        false, 0, store_address, store_ins->getInstructionAddress(), store_ins->getHWThread());
                    std_stores_in_flight.insert(store_req->getID());
                    memInterface->send(store_req);

                    return true;
                }
            } break;
            case MEM_TRANSACTION_LLSC_LOAD:
            {
                output->fatal(CALL_INFO, -1, "Error - attempted to issue a LLSC-load via store instruction. Invalid operation.\n");
            } break;
            case MEM_TRANSACTION_LLSC_STORE:
            {
                if(UNLIKELY(needs_split)) {
                    output->fatal(CALL_INFO, -1, "Error - attempted to perform an LLSC-store over a split-cache line. This is not permitted.\n");
                } else {
                    VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_STORE_FLG, "---> [memory-transaction]: LLSC-store store-at: 0x%" PRI_ADDR " width: %" PRIu64 "\n",
                        store_address, store_width);

                    store_req = new StandardMem::StoreConditional(store_address & address_mask, payload.size(), payload,
                                0, store_address, store_ins->getInstructionAddress(), store_ins->getHWThread() );
                }
            } break;
            case MEM_TRANSACTION_LOCK:
            {
                if(UNLIKELY(needs_split)) {
                    output->fatal(CALL_INFO, -1, "Error - attempted to perform an LOCK-store over a split-cache line. This is not permitted.\n");
                } else {
                    VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_STORE_FLG, "---> [memory-transaction]: LOCK-store store-at: 0x%" PRI_ADDR " width: %" PRIu64 "\n",
                        store_address, store_width);

                    store_req = new StandardMem::WriteUnlock(store_address & address_mask, payload.size(), payload,
                                0, store_address, store_ins->getInstructionAddress(), store_ins->getHWThread());
                }
            } break;
            }

            if (nullptr != store_req) {
                // equivalent to a seg-fault for the store
                if (store_address < 4096) {
                    store_ins->flagError();
                }

                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "-----> store-request sent to memory interface / entry marked dispatched\n");
                memInterface->send(store_req);
                store_entry->addRequest(store_req->getID());
                store_entry->markDispatched();
            } else {
                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_STORE_FLG, "-----> store-request was not sent to memory interface, record is nullptr\n");
            }

            return false;
        }

        virtual void addLoadRequest(VanadisLoadInstruction* load_ins,VanadisBasicLoadPendingEntry* load_entry, StandardMem::Request* load_req)
        {
            // load_entry->addRequest(load_req->getID(), load_ins->getSWThread());
            load_entry->addRequest(load_req->getID());
            // load_ins->setNumLoads(1);

        }

        void issueLoad(VanadisLoadInstruction* load_ins, uint64_t load_address, uint64_t load_width,
            VanadisBasicLoadPendingEntry* reuse_entry = nullptr) {
            StandardMem::Request* load_req = nullptr;

            #ifdef VANADIS_BUILD_DEBUG
            if ( isDbgInsAddr( load_ins->getInstructionAddress() ) || isDbgAddr( load_address ) ) {
                printf("%s() ins_addr=%#" PRIx64 " load_address=%#" PRIx64 " \n",__func__,load_ins->getInstructionAddress(), load_address);
            }
            #endif
            // An address the address space cannot hold is a fault, not a
            // request. See addressFitsSpace(). Refused here, before a pending
            // entry exists, so there is nothing to allocate and nothing to
            // unwind.
            if( UNLIKELY(! addressFitsSpace(load_address, load_width)) ) {
                noteAddressOutsideSpace("load", load_ins->getInstructionAddress(), load_address, load_width);
                load_ins->flagError();
                return;
            }

            // do we need to perform a split load (which loads from two cache lines)?
            const bool needs_split = operationStraddlesCacheLine(load_address, load_width);

            // The speculative queue already opened this load's record when the
            // load was renamed; the in-order one opens it here.
            VanadisBasicLoadPendingEntry* load_entry = (nullptr != reuse_entry)
                ? reuse_entry
                : new VanadisBasicLoadPendingEntry(load_ins, load_address, load_width);

            #if 0
            //with virtual memory we shouldn't need this but until we are sure we will leave it here
            if ( load_address + load_width < load_address || (load_address + load_width) & ~address_mask) {
                load_ins->markExecuted();
                return;
            }
            #endif

            switch (load_ins->getTransactionType()) {
                case MEM_TRANSACTION_NONE:
                {
                    if(UNLIKELY(needs_split)) {
                        VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_LOAD_FLG, "---> [memory-transaction]: standard load (line auto-split)\n");
                        // How many bytes are in the left most line?
                        const uint64_t load_width_right = (load_address + load_width) % cache_line_width;
                        assert(load_width_right > 0);

                        const uint64_t load_width_left = load_width - load_width_right;
                        assert(load_width_left > 0);

                        const uint64_t load_right_start = load_address + load_width_left;
                        assert((load_right_start % cache_line_width) == 0);

                        if(output->getVerboseLevel() >= 9) {
                            VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_LOAD_FLG, "---> split load at-left: 0x%" PRI_ADDR " left-width: %" PRIu64 " / at-right: 0x%" PRI_ADDR " right-width: %" PRIu64 "\n",
                                load_address, load_width_left, load_address + load_width_left, load_width_right);
                        }

                        load_req = new StandardMem::Read(load_address & address_mask, load_width_left, 0,
                            load_address, load_ins->getInstructionAddress(), load_ins->getHWThread());

                        addLoadRequest(load_ins,load_entry, load_req);
                        memInterface->send(load_req);

                        load_req = new StandardMem::Read((load_address + load_width_left) & address_mask, load_width_right, 0,
                            load_address + load_width_left, load_ins->getInstructionAddress(), load_ins->getHWThread());
                    } else {
                        if(output->getVerboseLevel() >= 9) {
                            VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_LOAD_FLG, "---> [memory-transaction]: standard load (not split) load-at: 0x%" PRI_ADDR " width: %" PRIu64 "\n",
                                load_address, load_width);
                        }

                        assert(load_width <= 8);
                        assert(load_width >= 0);

                        if(UNLIKELY(0 == (load_address & address_mask))) {
                            if(output->getVerboseLevel() >= 16) {
                                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "---> address resolves to zero, flag as error and do not generate event.\n");
                            }

                            load_ins->flagError();
                            load_req = nullptr;
                        } else {
                            load_req = new StandardMem::Read(load_address & address_mask, load_width, 0,
                                load_address, load_ins->getInstructionAddress(), load_ins->getHWThread());
                        }
                    }
                } break;
                case MEM_TRANSACTION_LLSC_LOAD:
                {
                    if(UNLIKELY(needs_split)) {
                        VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_LOAD_FLG, "---> load is marked LLSC but it requires a cache line split, generates an error\n");
                        load_ins->flagError();
                    } else {
                        VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_LOAD_FLG, "---> [memory-transaction]: LLSC-load (not split) load-at: 0x%" PRI_ADDR " width: %" PRIu64 "\n",
                            load_address, load_width);
                        load_req = new StandardMem::LoadLink(load_address & address_mask, load_width, 0,
                                            load_address, load_ins->getInstructionAddress(), load_ins->getHWThread());
                    }
                } break;
                case MEM_TRANSACTION_LLSC_STORE:
                {
                    output->fatal(CALL_INFO, -1,
                        "Error - logical error, LOAD instruction is marked with "
                        "an LLSC STORE transaction class.\n");
                } break;
                case MEM_TRANSACTION_LOCK:
                {
                    if(UNLIKELY(needs_split)) {
                        VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_LOAD_FLG, "---> load is marked LOCK but it requires a cache line split, this generates an error\n");
                        load_ins->flagError();
                    } else {
                        VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_LOAD_FLG, "---> [memory-transaction]: LOCK-load (not split) load-at: 0x%" PRI_ADDR " width: %" PRIu64 "\n",
                            load_address, load_width);
                        load_req = new StandardMem::ReadLock(load_address & address_mask, load_width, 0,
                                            load_address, load_ins->getInstructionAddress(), load_ins->getHWThread());
                    }
                } break;
            }

            // if the instruction does not trap an error we will continue to process it
            if(LIKELY(! load_ins->trapsError())) {


                assert(load_req != nullptr);

                addLoadRequest(load_ins,load_entry, load_req);
                memInterface->send(load_req);

                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "-----> ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " processed and requests sent to memory system. numRequests=%lu\n",
                    load_ins->getInstructionAddress(), load_ins->getHWThread(), load_entry->countRequests());

                loads_pending.push_back(load_entry);
            }
        }

        virtual bool sendLoadReq(VanadisLoadInstruction* load_ins)
        {
            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG,
                "In sendLoadReq (ScalarLSQ) hw_thr:%d\n", load_ins->getHWThread());
            std::vector<uint64_t> load_addresses;
            std::vector<uint16_t> load_widths;

            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, " (ScalarLSQ) -> load ins: 0x%" PRI_ADDR " / thr: %" PRIu32 "\n",
            load_ins->getInstructionAddress(), load_ins->getHWThread());
            bool result = load_process(load_ins->getHWThread(),load_ins,
                    load_addresses, load_widths);

            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, " (ScalarLSQ) -> load ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " result=%s #load_address=%lu #load_widths=%lu...\n",
                    load_ins->getInstructionAddress(), load_ins->getHWThread(), (result==true) ? "success":"fail", load_addresses.size(), load_widths.size());

            if(LIKELY((load_addresses.size()>0) & (load_widths.size()>0)))
            {
                issueLoad(load_ins, load_addresses[0], load_widths[0]);
            }
            return result;
        }

        virtual bool sendStoreReq(VanadisInstruction* store_ins_temp)
        {
            VanadisStoreInstruction* store_ins = store_ins_temp->asStore();
            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG,
                "In sendstoreReq (ScalarLSQ) hw_thr:%d\n", store_ins->getHWThread());
            uint64_t store_address_last = 0;
            uint8_t trap_error = 0;
            VanadisBasicStorePendingEntry* new_pending_store= store_process(store_ins->getHWThread(),store_ins,&store_address_last, &trap_error);
            if(trap_error==1)
            {
                ;
            }
            else
            {
                if(new_pending_store==nullptr)
                {
                    output->fatal(CALL_INFO, -1, "Error: store process failed (ins: 0x%" PRI_ADDR ", thr: %" PRIu32 ")\n",
                        store_ins->getInstructionAddress(), store_ins->getHWThread());
                }
                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, " (ScalarLSQ) -> queue front is store: ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " has issued so will process...\n",
                        new_pending_store->getStoreInstruction()->getInstructionAddress(), new_pending_store->getStoreInstruction()->getHWThread());
                stores_pending[store_ins->getHWThread()].push_back(new_pending_store);
                stores_pending_size++;
            }
            return true;
        }

        // ===================================================================
        // THE SPECULATIVE PIPELINE
        // ===================================================================
        //
        // Five functions, and between them they are the whole of what a modern
        // core's memory pipeline does that the in-order queue above does not.
        //
        //   resolveStore        a store's address becomes known
        //   noteStoreResolved   ...and every younger load that has already read
        //                       overlapping bytes is flagged
        //   resolveLoad         a load's address becomes known
        //   searchOlderStores   ...and it decides what to do about the stores
        //                       ahead of it: go, wait, or take their bytes
        //   forwardToRegister   taking their bytes

        /// A store's address and value are known. Both arrive together, because
        /// this core issues an instruction only when every operand is ready:
        /// there is no separate store-address operation here, so a store still
        /// waiting for its DATA looks exactly like a store whose ADDRESS is
        /// unknown, and the predictor is consulted for it. That is conservative
        /// and never wrong.
        void resolveStore(VanadisStoreInstruction* store_ins)
        {
            const uint32_t thr = store_ins->getHWThread();

            auto reserved = reserved_stores_.find(store_ins);
            if( UNLIKELY(reserved == reserved_stores_.end()) ) {
                output->fatal(CALL_INFO, -1,
                    "Error: store ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " reached the queue without a reserved slot.\n",
                    store_ins->getInstructionAddress(), thr);
            }

            VanadisBasicStorePendingEntry* entry = reserved->second;
            reserved_stores_.erase(reserved);

            uint64_t store_address = 0;
            uint16_t store_width   = 0;
            store_ins->computeStoreAddress(output, registerFiles->at(thr), &store_address, &store_width);

            if( UNLIKELY(store_ins->trapsError()) ) {
                // Nothing will ever be sent for it, so it must not sit in the
                // queue holding younger loads back. The core stops on it if it
                // retires.
                store_ins->markExecuted();
                removeStoreEntry(thr, entry);
                return;
            }

            if( UNLIKELY((store_ins->getTransactionType() == MEM_TRANSACTION_LLSC_STORE)
                      || (store_ins->getTransactionType() == MEM_TRANSACTION_LOCK)) ) {
                ordered_ins_[thr] = store_ins;
            }

            entry->resolve(store_address, store_width, store_ins->getValueRegisterType(),
                store_ins->getValueRegister());
            entry->setSWThr(thr);
            entry->addThr(thr);

            // THE BYTES, kept so a younger load this store covers can be
            // answered without waiting for the store to reach memory. Read out
            // of the same register, at the same offset, with the same call the
            // store itself will use when it issues.
            if( LIKELY((store_ins->getTransactionType() == MEM_TRANSACTION_NONE)
                    && (store_width <= VANADIS_LSQ_MAX_FWD_BYTES)) ) {
                uint8_t  bytes[VANADIS_LSQ_MAX_FWD_BYTES];
                uint16_t value_thread = 0;
                uint16_t value_reg    = 0;
                getStoreTarget(entry, store_ins, &value_thread, &value_reg);
                registerFiles->at(value_thread)->copyFromRegister(value_reg, store_ins->getRegisterOffset(),
                    &bytes[0], store_width, store_ins->getValueRegisterType() == STORE_FP_REGISTER);
                entry->setForwardData(bytes, store_width);
            } else {
                entry->clearForwardData();
            }

            noteStoreResolved(thr, entry);
        }

        /// THE ORDERING VIOLATION, detected at the one instant it can be: the
        /// moment a store's address becomes known. Every younger load of the
        /// same thread that has already read bytes this store writes read the
        /// wrong ones.
        void noteStoreResolved(const uint32_t thr, VanadisBasicStorePendingEntry* store_entry)
        {
            const uint64_t store_age = store_entry->getAge();
            const uint64_t store_pc  = store_entry->getInstructionAddress();

            // Ages increase towards the back, so walk back to front and stop as
            // soon as the entries are older than the store.
            for( size_t i = load_q[thr].size(); i-- > 0; ) {
                VanadisBasicLoadPendingEntry* load_entry = load_q[thr][i];

                if( load_entry->getAge() < store_age ) { break; }

                const auto state = load_entry->getState();
                if( (state != VanadisBasicLoadPendingEntry::INFLIGHT)
                 && (state != VanadisBasicLoadPendingEntry::DONE) ) { continue; }

                if( load_entry->hasViolated() ) { continue; }

                // A load answered from a store YOUNGER than this one saw the
                // value that wins; it did not miss anything. A load answered by
                // memory records age 0 and always fails this test.
                if( store_age <= load_entry->forwardedFrom() ) { continue; }

                if( ! store_entry->storeAddressOverlaps(load_entry->getLoadAddress(),
                        load_entry->getLoadWidth()) ) { continue; }

                load_entry->markViolated(store_pc);
                load_entry->getLoadInstruction()->markReplay();
                stat_mem_violations->addData(1);
            }
        }

        /// A load's address is known. From here it is the load's own decision
        /// what to do about the stores ahead of it.
        void resolveLoad(VanadisLoadInstruction* load_ins)
        {
            const uint32_t thr = load_ins->getHWThread();

            auto reserved = reserved_loads_.find(load_ins);
            if( UNLIKELY(reserved == reserved_loads_.end()) ) {
                output->fatal(CALL_INFO, -1,
                    "Error: load ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " reached the queue without a reserved slot.\n",
                    load_ins->getInstructionAddress(), thr);
            }

            VanadisBasicLoadPendingEntry* entry = reserved->second;
            reserved_loads_.erase(reserved);

            uint64_t load_address = 0;
            uint16_t load_width   = 0;
            load_ins->computeLoadAddress(output, registerFiles->at(thr), &load_address, &load_width);

            if( UNLIKELY(load_ins->trapsError()) ) {
                // Nothing is sent and nothing will answer, so it must not be
                // left waiting. The pipeline stops on it if it retires.
                entry->setState(VanadisBasicLoadPendingEntry::DONE);
                return;
            }

            if( UNLIKELY((load_ins->getTransactionType() == MEM_TRANSACTION_LLSC_LOAD)
                      || (load_ins->getTransactionType() == MEM_TRANSACTION_LOCK)) ) {
                ordered_ins_[thr] = load_ins;
            }

            entry->resolve(load_address, load_width);
            searchOlderStores(thr, entry);

            if( VanadisBasicLoadPendingEntry::WAITING == entry->getState() ) { waiting_loads_[thr]++; }
        }

        /// The load looks back over the stores ahead of it, youngest first, and
        /// stops at the first one that decides the matter.
        void searchOlderStores(const uint32_t thr, VanadisBasicLoadPendingEntry* load_entry)
        {
            VanadisLoadInstruction* load_ins = load_entry->getLoadInstruction();
            auto&                   store_q  = stores_pending[thr];

            const uint64_t load_address = load_entry->getLoadAddress();
            const uint64_t load_width   = load_entry->getLoadWidth();
            const uint64_t load_age     = load_entry->getAge();
            const uint64_t load_pc      = load_ins->getInstructionAddress();

            // How many entries at the front of the store queue are older than
            // this load. Ages increase towards the back.
            size_t older = 0;
            while( (older < store_q.size()) && (store_q[older]->getAge() < load_age) ) { older++; }

            // A load-linked or a locked load is a synchronisation instruction,
            // not an ordinary read: it never passes a store and is never
            // answered from one. This is what load_process() asks for the
            // in-order queue, said against ages instead of against emptiness.
            if( UNLIKELY((load_ins->getTransactionType() == MEM_TRANSACTION_LLSC_LOAD)
                      || (load_ins->getTransactionType() == MEM_TRANSACTION_LOCK)) ) {
                if( older > 0 ) { return; }
                sendLoadFromEntry(load_entry);
                return;
            }

            // A load that has just been replayed waits for every older store,
            // whatever the predictor thinks. See noteReplay().
            const bool forced = forced_hold_armed_[thr] && (forced_hold_pc_[thr] == load_pc);

            bool speculated = false;

            for( size_t i = older; i-- > 0; ) {
                VanadisBasicStorePendingEntry* store_entry = store_q[i];

                if( ! store_entry->isResolved() ) {
                    // Nobody knows whether this store touches the load. Ask.
                    const VanadisBasicOlderStoreView view(store_q, load_age);
                    const bool predictor_go = mem_dep_[thr]->speculate(load_pc, view);

                    if( forced || (! predictor_go) ) {
                        // Counted only when it is the PREDICTOR holding the
                        // load, so that the statistic says what the table is
                        // doing and not what the safety net is doing.
                        if( (! predictor_go) && (! load_entry->wasHeld()) ) {
                            load_entry->markHeld();
                            stat_mem_predictor_holds->addData(1);
                        }
                        return;                 // stays WAITING, retried next cycle
                    }

                    speculated = true;
                    continue;
                }

                if( ! store_entry->storeAddressOverlaps(load_address, load_width) ) { continue; }

                if( store_entry->fullyCovers(load_address, load_width)
                 && canForwardFrom(load_ins, store_entry) ) {
                    if( speculated ) {
                        load_entry->markSpeculated();
                        stat_mem_loads_speculated->addData(1);
                    }
                    forwardToRegister(load_entry, store_entry);
                    load_entry->setForwardedFrom(store_entry->getAge());
                    load_entry->setState(VanadisBasicLoadPendingEntry::DONE);
                    load_ins->markExecuted();
                    stat_loads_executed->addData(1);
                    stat_mem_loads_forwarded->addData(1);
                    return;
                }

                // The store writes some of the load's bytes but not all of
                // them, or it is a store nothing may be forwarded from. Wait
                // for it to reach memory and read it back, which is what the
                // in-order queue does for every overlap.
                return;
            }

            if( speculated ) {
                load_entry->markSpeculated();
                stat_mem_loads_speculated->addData(1);
            }
            sendLoadFromEntry(load_entry);
        }

        /// What may be answered out of a store queue entry. Deliberately narrow.
        /// A floating-point destination is excluded because the load response
        /// path writes one differently, and two pieces of code that must agree
        /// byte for byte are better replaced by one that is never taken.
        bool canForwardFrom(VanadisLoadInstruction* load_ins, VanadisBasicStorePendingEntry* store_entry) const
        {
            if( ! forward_ ) { return false; }
            if( load_ins->getValueRegisterType() != LOAD_INT_REGISTER ) { return false; }
            if( load_ins->getTransactionType() != MEM_TRANSACTION_NONE ) { return false; }
            if( store_entry->getStoreInstruction()->getTransactionType() != MEM_TRANSACTION_NONE ) { return false; }
            if( store_entry->isDispatched() ) { return false; }
            if( ! store_entry->canForward() ) { return false; }
            if( load_ins->getLoadWidth() > VANADIS_LSQ_MAX_FWD_BYTES ) { return false; }
            return true;
        }

        /// The bytes of an older store, put into the load's destination register
        /// with the sign extension the load asked for. This is the read-response
        /// path's integer case with the response replaced by the store's copy of
        /// what it is going to write, and one request rather than two, so there
        /// is no split to reassemble.
        void forwardToRegister(VanadisBasicLoadPendingEntry* load_entry,
            VanadisBasicStorePendingEntry* store_entry)
        {
            VanadisLoadInstruction* load_ins = load_entry->getLoadInstruction();

            if( UNLIKELY(load_ins->trapsError()) ) { return; }

            const uint32_t thr        = load_ins->getHWThread();
            const uint16_t target_reg = load_ins->getPhysIntRegOut(0);

            if( target_reg == load_ins->getISAOptions()->getRegisterIgnoreWrites() ) { return; }

            const uint64_t load_width  = load_entry->getLoadWidth();
            const uint64_t reg_offset  = load_ins->getRegisterOffset();
            const uint64_t byte_offset = load_entry->getLoadAddress() - store_entry->getStoreAddress();
            const uint8_t* source      = store_entry->forwardData() + byte_offset;

            const uint32_t reg_width = registerFiles->at(thr)->getIntRegWidth();

            if( UNLIKELY((reg_offset + load_width) > reg_width) ) {
                output->fatal(CALL_INFO, -1,
                    "store-to-load forward does not fit the register: ins 0x%" PRI_ADDR " (%s), width %" PRIu64
                    ", reg-offset %" PRIu64 ", reg-width %" PRIu32 "\n",
                    load_ins->getInstructionAddress(), load_ins->getInstCode(), load_width, reg_offset, reg_width);
            }

            std::vector<uint8_t> register_value(reg_width);
            registerFiles->at(thr)->copyFromIntRegister(target_reg, 0, &register_value[0], reg_width);

            for( uint64_t i = 0; i < load_width; ++i ) {
                register_value.at(reg_offset + i) = source[i];
            }

            const uint8_t fill = (load_ins->performSignExtension()
                && ((register_value.at(reg_offset + load_width - 1) & 0x80) != 0)) ? 0xFF : 0x00;

            for( uint64_t i = reg_offset + load_width; i < reg_width; ++i ) {
                register_value.at(i) = fill;
            }

            registerFiles->at(thr)->copyToIntRegister(target_reg, 0, &register_value[0], register_value.size());

            VANADIS_VERB(output, 9, VANADIS_DBG_LSQ_LOAD_FLG,
                "---> LSQ forward: load 0x%" PRI_ADDR " <- store 0x%" PRI_ADDR " (addr 0x%" PRI_ADDR ", width %" PRIu64 ")\n",
                load_ins->getInstructionAddress(), store_entry->getInstructionAddress(),
                load_entry->getLoadAddress(), load_width);
        }

        /// Send the load to memory out of its queue entry. A refusal -- an
        /// address the space cannot hold, an address that masks to zero -- sends
        /// nothing and leaves no request outstanding, so the entry is finished
        /// rather than left waiting for an answer that will not come.
        void sendLoadFromEntry(VanadisBasicLoadPendingEntry* load_entry)
        {
            VanadisLoadInstruction* load_ins = load_entry->getLoadInstruction();

            load_entry->setState(VanadisBasicLoadPendingEntry::INFLIGHT);
            issueLoad(load_ins, load_entry->getLoadAddress(), load_entry->getLoadWidth(), load_entry);

            if( UNLIKELY(0 == load_entry->countRequests()) ) {
                load_entry->setState(VanadisBasicLoadPendingEntry::DONE);
            }
        }

        /// One cycle's work for one hardware thread: a fence if one is waiting
        /// and its conditions are met, then the oldest load that can now go.
        void attempt_to_issue_speculative(const uint32_t thr)
        {
            // A FENCE. In this mode op_q holds nothing else: loads and stores
            // are resolved where they are pushed. The core will not select
            // anything younger than a fence it has not executed, so every
            // resolved load and every resolved store in the queue right now is
            // OLDER than the fence -- which is what makes these two tests, which
            // name no ages, the right ones.
            if( UNLIKELY(! op_q[thr].empty()) ) {
                VanadisBasicLoadStoreEntry* fence_entry = op_q[thr].front();
                VanadisFenceInstruction*    fence_ins   = fence_entry->getInstruction()->asFence();

                bool can_execute = true;

                if( fence_ins->createsLoadFence() ) {
                    can_execute = ! anyLoadOutstanding(thr);
                }

                if( fence_ins->createsStoreFence() ) {
                    can_execute = can_execute && (! anyStoreResolved(thr)) && (std_stores_in_flight.size() == 0);
                }

                if( can_execute ) {
                    fence_ins->markExecuted();
                    stat_fences_executed->addData(1);
                    delete fence_entry;
                    op_q[thr].pop_front();
                    op_q_size--;
                }

                // A load older than the fence may still be waiting on a store,
                // and the fence is waiting for exactly that load, so the walk
                // below still runs.
            }

            if( 0 == waiting_loads_[thr] ) { return; }

            unsigned examined = 0;

            for( auto* load_entry : load_q[thr] ) {
                if( VanadisBasicLoadPendingEntry::WAITING != load_entry->getState() ) { continue; }
                if( ++examined > max_waiting_walk ) { break; }

                searchOlderStores(thr, load_entry);

                if( VanadisBasicLoadPendingEntry::WAITING != load_entry->getState() ) {
                    waiting_loads_[thr]--;
                    return;             // one load per attempt, as issues_per_cycle says
                }
            }
        }

        /// True while any load of this thread still owes a value.
        bool anyLoadOutstanding(const uint32_t thr) const
        {
            for( auto* entry : load_q[thr] ) {
                const auto state = entry->getState();
                if( (VanadisBasicLoadPendingEntry::WAITING == state)
                 || (VanadisBasicLoadPendingEntry::INFLIGHT == state) ) { return true; }
            }
            return false;
        }

        /// True while any store of this thread has an address and has not yet
        /// reached memory. Reserved entries are younger than any fence that can
        /// be asking, so they do not count.
        bool anyStoreResolved(const uint32_t thr) const
        {
            for( auto* entry : stores_pending[thr] ) {
                if( entry->isResolved() ) { return true; }
            }
            return false;
        }

        void dropFromLoadsPending(VanadisBasicLoadPendingEntry* entry)
        {
            for( auto itr = loads_pending.begin(); itr != loads_pending.end(); ++itr ) {
                if( (*itr) == entry ) { loads_pending.erase(itr); return; }
            }
        }

        void removeStoreEntry(const uint32_t thr, VanadisBasicStorePendingEntry* entry)
        {
            for( auto itr = stores_pending[thr].begin(); itr != stores_pending[thr].end(); ++itr ) {
                if( (*itr) == entry ) {
                    stores_pending[thr].erase(itr);
                    stores_pending_size--;
                    delete entry;
                    return;
                }
            }
        }

        bool attempt_to_issue(uint64_t cycle, uint16_t attempt_this_cycle, int thr)
        {
            // if we don't have any work to do, return and get out of here
            if(0 == op_q[thr].size()) {
                return false;
            }
            VanadisBasicLoadStoreEntry* front_entry = op_q[thr].front();
            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "-> cycle: %" PRIu64 " / attempt: %" PRIu16 " / thr: %" PRId32"\n", cycle, attempt_this_cycle, thr);



            if(! front_entry->getInstruction()->completedIssue()) {
                if(output->getVerboseLevel() >= 16) {
                    VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " has not completed issue, will not process this cycle.\n",
                        front_entry->getInstruction()->getInstructionAddress(), front_entry->getInstruction()->getHWThread());
                }
                return false;
            }

            switch(front_entry->getEntryOp()) {
                case VanadisBasicLoadStoreEntryOp::LOAD:
                {
                    //VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> cycle: %" PRIu64 " attempt to issue LOAD\n", cycle);
                    VanadisLoadInstruction* load_ins =
                        front_entry->getInstruction()->asLoad();

                    if(UNLIKELY(load_ins == nullptr))
                    {
                        output->fatal(CALL_INFO, -1, "Error: attempted to convert a load entry to a load instruction but failed, ins: 0x%" PRI_ADDR " / thr: %" PRIu32 "\n",
                            front_entry->getInstructionAddress(), front_entry->getHWThread());
                    }

                    // can't do anything this cycle, so return false
                    if(loads_pending.size() >= max_loads)
                    {
                        VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> cycle: %" PRIu64 " issue LOAD failed: max_loads\n", cycle);
                        return false;
                    }

                    VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "-> queue front is load: ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " has issued so will process...\n",
                        load_ins->getInstructionAddress(), load_ins->getHWThread());

                    bool result = sendLoadReq(load_ins);

                    if(result)
                    {
                        // pop front entry and tell the caller we did something (true)
                        delete op_q[thr].front();
                        op_q[thr].pop_front();
                        op_q_size--;
                        //VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> cycle: %" PRIu64 " issue LOAD succeeded\n", cycle);
                        return true;
                    }
                    return result;
                } break;
                case VanadisBasicLoadStoreEntryOp::STORE:
                {

                    //VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> cycle: %" PRIu64 " attempt to issue STORE\n", cycle);
                    // VanadisStoreInstruction* store_ins = dynamic_cast<VanadisStoreInstruction*>(
                    //     front_entry->getInstruction());
                    VanadisInstruction* store_ins = front_entry->getInstruction();
                    if(UNLIKELY(store_ins == nullptr)) {
                        output->fatal(CALL_INFO, -1, "Error: attempted to convert a store entry into store instruction but this failed (ins: 0x%" PRI_ADDR ", thr: %" PRIu32 ")\n",
                            front_entry->getInstructionAddress(), front_entry->getHWThread());
                    }

                    // if we have too many operations pending, return false to tell handler
                    // we couldn't perform any operations this cycle
                    if(stores_pending_size >= max_stores) {
                        //VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> cycle: %" PRIu64 " issue STORE failed: max stores\n", cycle);
                        return false;
                    }

                    if(output->getVerboseLevel() >= 16) {
                        VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "-> queue front is store: ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " has issued so will process...\n",
                            front_entry->getInstructionAddress(), front_entry->getHWThread());
                    }

                    sendStoreReq(store_ins);
                    // clear the front entry as we have just processed it
                    delete op_q[thr].front();
                    op_q[thr].pop_front();
                    op_q_size--;
                    //VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> cycle: %" PRIu64 " issue STORE succeeded\n", cycle);
                    return true;
                } break;
                case VanadisBasicLoadStoreEntryOp::FENCE:
                {
                    VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> cycle: %" PRIu64 " attempt to issue FENCE\n", cycle);
                    VanadisInstruction* current_ins = op_q[thr].front()->getInstruction();
                    VanadisFenceInstruction* current_fence_ins = current_ins->asFence();

                    bool can_execute = true;

                    // fence instruction can be executed IF there are no pending loads IF it fences loads
                    // AND there are no pending stores IF if fences stores
                    if(current_fence_ins->createsLoadFence()) {
                        // if no pending loads, then we are good to go
                        can_execute = (!pendingLoads(current_fence_ins->getHWThread()));
                    }

                    if(current_fence_ins->createsStoreFence()) {
                        // stores are fenced if there are no pending stores AND all issued to the memory system
                        // have returned so are currently visible.
                        can_execute = can_execute && (stores_pending[current_fence_ins->getHWThread()].size() == 0) &&
                            (std_stores_in_flight.size() == 0);
                    }

                    if(can_execute) {
                        // if(output->getVerboseLevel() >= 16)
                        {
                            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "-> execute fence instruction (0x%" PRI_ADDR "), all checks have passed.\n",
                                current_fence_ins->getInstructionAddress());
                        }
                        current_fence_ins->markExecuted();
                        stat_fences_executed->addData(1);

                        // erase the front entry
                        delete op_q[thr].front();
                        op_q[thr].pop_front();
                        op_q_size--;
                        VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> cycle: %" PRIu64 " issue FENCE succeeded\n", cycle);
                        return true;
                    } else {
                        VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> cycle: %" PRIu64 " issue FENCE failed: cannot execute\n", cycle);
                        return false;
                    }
                } break;
            }

            // default is do not call me again
            return false;
        }

        bool load_process(uint32_t sw_thr,VanadisLoadInstruction* load_ins,
                        std::vector<uint64_t>& load_addresses, std::vector<uint16_t>& load_widths )
        {
            VanadisRegisterFile* hw_thr_reg = registerFiles->at(sw_thr);
            uint64_t load_address = 0;
            uint16_t load_width   = 0;
            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "---> computeLoadAddress for sw_thr: %" PRIu32 "\n",sw_thr);
            load_ins->computeLoadAddress(output, hw_thr_reg, &load_address, &load_width);
            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "---> computeLoadAddress for 0x%" PRI_ADDR " / sw_thr: %" PRIu32 "\n",
                load_address, sw_thr);

            if(UNLIKELY(load_ins->trapsError()))
            {
                // if(output->getVerboseLevel() >= 16)
                {
                    VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "---> load ins: 0x%" PRI_ADDR " / hw_thr: %" PRIu32 "sw_thr: %" PRIu32 " traps error, will not process and allow pipeline to handle \n",
                        load_ins->getInstructionAddress(), load_ins->getHWThread(), sw_thr);
                    // load_ins->setNumLoads(0);
                    load_addresses.clear();
                    load_widths.clear();
                    return true;
                }
            }
            else
            {
                if(output->getVerboseLevel() >= 16)
                {
                    VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "---> load ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " want load at 0x%" PRI_ADDR " / width: %" PRIu16 "\n",
                        load_ins->getInstructionAddress(), load_ins->getHWThread(), load_address, load_width);
                }

                // check to see if loading from this address would conflict with a store which
                // we have pending, if yes, wait for conflict to clear and then we can proceed
                if(UNLIKELY(checkStoreConflict(load_ins->getHWThread(), load_address, load_width)))
                {
                    if(output->getVerboseLevel() >= 16)
                    {
                        VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "---> load ins: 0x%" PRI_ADDR " / thr: %" PRIu32 " conflicts with store entry, will not issue until conflict is resolved (load-addr: 0x%" PRI_ADDR " / width: %" PRIu32 ")\n",
                            load_ins->getInstructionAddress(), load_ins->getHWThread(), load_address, load_width);
                    }

                    // tell caller we would not issue
                    VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "---> issue LOAD failed: store conflict\n");
                    return false;

                    // Drain store q to ensure that a paired SC/Unlock can be issued close to the LL/Lock
                }
                else if((load_ins->getTransactionType() == MEM_TRANSACTION_LLSC_LOAD) || (load_ins->getTransactionType() == MEM_TRANSACTION_LOCK))
                {
                    if (!stores_pending[load_ins->getHWThread()].empty())
                    {
                        VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "---> issue LLSC/LOCK LOAD failed: store pending\n");
                        return false;
                    }
                    else
                    {
                        VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "---> issue LLSC/LOCK LOAD possible sw_thr=%d\n", sw_thr);
                        // issueLoad(load_ins, load_address, load_width);
                        load_addresses.push_back(load_address);
                        load_widths.push_back(load_width);
                    }
                }
                else
                {
                    // We are good to issue with all checks completed!
                    // issueLoad(load_ins, load_address, load_width);
                    VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> issue LOAD possible sw_thr=%d\n", sw_thr);
                    load_addresses.push_back(load_address);
                    load_widths.push_back(load_width);
                }
            }
            return true;
        }

        VanadisBasicStorePendingEntry* store_process(uint32_t sw_thr,VanadisStoreInstruction* store_ins,uint64_t* store_address_last, uint8_t* trap_error)
        {
            // registerFiles->at(sw_thr)->setTID(sw_thr); // reference for when calculating load address;
            VanadisRegisterFile* hw_thr_reg = registerFiles->at(sw_thr);

            uint64_t store_address = 0;
            uint16_t store_width  = 0;
            *trap_error = 0;
            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "--> computeStoreAddress for sw_thr: %" PRIu32 "\n",sw_thr);
            store_ins->computeStoreAddress(output, hw_thr_reg, &store_address, &store_width);
            if(store_ins->trapsError())
            {
                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "----> warning: 0x%" PRI_ADDR " / thr: %" PRIu32 " traps error, marks executed and does not process.\n",
                    store_ins->getInstructionAddress(), store_ins->getHWThread());
                // store_ins->setNumStores(0);
                store_ins->markExecuted();
                *store_address_last = store_address;
                *trap_error=1;
            }
            VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "----> computed store address: 0x%" PRI_ADDR " store_address_last: 0x%" PRI_ADDR " width: %" PRIu16 " sw_thr: %" PRIu32 " hw_thr: %" PRIu32 " ins_addr: 0x%" PRI_ADDR "\n",
                store_address, *store_address_last, store_width, sw_thr, store_ins->getHWThread(),store_ins->getInstructionAddress());
            if ((*store_address_last != store_address) || (*store_address_last ==0))
            {
                VanadisBasicStorePendingEntry* new_pending_store = new VanadisBasicStorePendingEntry(store_ins, store_address, store_width,
                                                store_ins->getValueRegisterType(),store_ins->getValueRegister());
                new_pending_store->setSWThr(sw_thr);
                new_pending_store->addThr(sw_thr);

                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "----> ins_addr: 0x%" PRI_ADDR " store address: 0x%" PRI_ADDR " store_address_last: 0x%" PRI_ADDR " width: %" PRIu16 " sw_thr: %" PRIu32 " hw_thr: %" PRIu32 "\n",
                    new_pending_store->getStoreInstruction()->getInstructionAddress(), store_address, *store_address_last, store_width, sw_thr, store_ins->getHWThread());
                *store_address_last = store_address;
                return new_pending_store;
            }
            else
            {
                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "----> pending_store=null address: 0x%" PRI_ADDR " store_address_last: 0x%" PRI_ADDR " width: %" PRIu16 " sw_thr: %" PRIu32 " hw_thr: %" PRIu32 "\n",
                    store_address, *store_address_last, store_width, sw_thr, store_ins->getHWThread());
                return nullptr;
            }

        }

        // AN ACCESS THAT DOES NOT FIT THE ADDRESS SPACE THIS CORE IS CONFIGURED FOR.
        //
        // `address_mask` is this core's declaration of how wide an address is:
        // a machine whose guests live below 4 GiB is given 0xFFFFFFFF, and the
        // request this queue builds carries `address & address_mask` while the
        // queue's own pending entry keeps the address the instruction computed.
        // For every access the address space can hold those two are the same
        // number and the mask does nothing.
        //
        // For one it cannot hold they are different numbers, and the difference
        // is not recoverable later: the response comes back carrying the masked
        // address, the read-response handler subtracts the entry's unmasked one
        // to find where in the register the bytes belong, the subtraction
        // underflows to about 2^64, and the run dies with a message about a
        // register rather than about an address.
        //
        // Such an access is a fault, not a request. An out-of-order core runs
        // instructions from a branch it has predicted and may be wrong about,
        // so it computes addresses from register values that belong to another
        // iteration or to no iteration at all -- an index that is a not-visited
        // sentinel, a pointer read one element past an array -- and a load with
        // one of those is normal, is squashed when the misprediction is
        // discovered, and must not decide the outcome of the simulation. A real
        // machine answers it with a translation fault the squash discards. This
        // one refuses to send it and flags the instruction, which has exactly
        // that shape: nothing happens if the instruction is squashed, and the
        // core stops on it, naming it, if it ever reaches the head of the
        // reorder buffer and is therefore an access the program really makes.
        //
        // The check is on both ends of the access, because an access may begin
        // inside the space and end outside it.
        bool addressFitsSpace(uint64_t address, uint64_t width) const
        {
            if( 0 == width ) { return true; }

            const uint64_t last = address + width - 1;

            // The access wraps the top of the 64-bit space; no address space
            // holds it whatever the mask is.
            if( last < address ) { return false; }

            return ((address & address_mask) == address) && ((last & address_mask) == last);
        }

        // Said once per core per run, because a program that has one of these
        // usually has thousands and they are all the same event. The count is
        // the durable record; this line is so that a reader of the log knows to
        // go and look at it.
        void noteAddressOutsideSpace(const char* op, uint64_t ins_addr, uint64_t address, uint64_t width)
        {
            stat_addr_outside_space->addData(1);

            if( ! said_addr_outside_space ) {
                said_addr_outside_space = true;
                output->verbose(CALL_INFO, 0, 0,
                    "note: a %s at 0x%" PRI_ADDR " (width %" PRIu64 ") from instruction 0x%" PRI_ADDR
                    " lies outside the address space this core is configured for (address_mask 0x%"
                    PRI_ADDR "). It is not sent to memory and the instruction is flagged; if it is a"
                    " speculated instruction the squash discards it, and if it retires the core stops"
                    " on it. Further occurrences are counted in the addr_outside_space statistic and"
                    " not printed.\n",
                    op, address, width, ins_addr, address_mask);
            }
        }

        bool operationStraddlesCacheLine(uint64_t address, uint64_t width) const
        {
            const uint64_t cache_line_left  = (address / cache_line_width);
            const uint64_t cache_line_right = ((address + width - 1) / cache_line_width);

            const bool splits_line = cache_line_left != cache_line_right;

            if(output->getVerboseLevel() >= 16) {
                VANADIS_VERB(output, 16, VANADIS_DBG_LSQ_LOAD_FLG, "---> check split addr: %" PRIu64 " (0x%" PRI_ADDR ") / width: %" PRIu64 " / cache-line: %" PRIu64 " / line-left: %" PRIu64 " / line-right: %" PRIu64 " / split: %3s\n",
                    address, address, width, cache_line_width, cache_line_left, cache_line_right, splits_line ? "yes" : "no");
            }

            return splits_line;
        }

        void copyPayload(std::vector<uint8_t>& buffer, uint8_t* reg, uint16_t offset, uint16_t length) const
        {
            for(uint16_t i = 0; i < length; ++i) {
                buffer.push_back(reg[offset + i]);
            }
        }

        bool pendingStores(const uint32_t thr)
        {
            return stores_pending[thr].size() > 0;
        }

        bool pendingLoads(const uint32_t thr)
        {
            bool matchID = false;

            for(auto load_itr = loads_pending.begin(); load_itr != loads_pending.end(); load_itr++) {
                if((*load_itr)->getHWThread() == thr) {
                    matchID = true;
                    break;
                }
            }

            return matchID;
        }

        bool checkStoreConflict(const uint32_t thread, const uint64_t address, const uint64_t width)
        {
            bool conflicts = false;

            for(auto store_itr = stores_pending[thread].begin(); store_itr != stores_pending[thread].end(); store_itr++) {
                VanadisBasicStorePendingEntry* current_entry = (*store_itr);

                if(UNLIKELY(current_entry->storeAddressOverlaps(address, width))) {
                    conflicts = true;
                    break;
                }
            }

            return conflicts;
        }


        // Per-hardware-thread queues
        std::vector< std::deque<VanadisBasicLoadStoreEntry*> > op_q;
        std::vector< std::deque<VanadisBasicStorePendingEntry*> > stores_pending;
        std::deque<VanadisBasicLoadPendingEntry*> loads_pending;
        std::set<StandardMem::Request::id_t> std_stores_in_flight;
        int op_q_index; // Next hw_thread to check in op_q queues
        int stores_pending_index; // Next hw thread to check in stores_pending q's
        size_t op_q_size;
        size_t stores_pending_size;

        StandardMem* memInterface;
        StandardMemHandlers* std_mem_handlers;

        const size_t max_stores;
        const size_t max_loads;

        const uint32_t max_issue_attempts_per_cycle;

        // ---- the speculative queue's state ----------------------------------
        //
        // load_q is the load queue: one entry per load from the moment it is
        // renamed until the moment it retires, oldest at the front. The store
        // queue is stores_pending, which now also holds the slots of stores
        // whose addresses are not known yet. next_age_ orders the two against
        // each other; it is reset when the thread's queues are cleared, which
        // removes everything an age could be compared against.
        bool                                                  spec_ = false;
        bool                                                  forward_ = true;
        std::string                                           mdp_kind_;
        std::vector< std::deque<VanadisBasicLoadPendingEntry*> > load_q;
        std::vector<VanadisInstruction*>                      ordered_ins_;
        size_t                                                load_q_size = 0;
        std::vector<uint64_t>                                 next_age_;
        std::vector<size_t>                                   waiting_loads_;
        std::vector<VanadisMemDepPredictor*>                  mem_dep_;
        std::vector<uint64_t>                                 forced_hold_pc_;
        std::vector<bool>                                     forced_hold_armed_;
        std::unordered_map<VanadisInstruction*, VanadisBasicLoadPendingEntry*>  reserved_loads_;
        std::unordered_map<VanadisInstruction*, VanadisBasicStorePendingEntry*> reserved_stores_;

        // How many waiting loads one issue attempt will look at. A load held by
        // the predictor is looked at again every cycle, and without a bound a
        // queue full of them would cost the simulator a walk of 192 entries per
        // attempt to find the one that can move.
        static const unsigned max_waiting_walk = 32;

        uint64_t cache_line_width;
        uint64_t address_mask;
        bool     said_addr_outside_space = false;

        Statistic<uint64_t>* stat_store_buffer_entries;
        Statistic<uint64_t>* stat_op_q_size;
        Statistic<uint64_t>* stat_stores_pending;
        Statistic<uint64_t>* stat_loads_pending;
        Statistic<uint64_t>* stat_addr_outside_space;
        Statistic<uint64_t>* stat_stores_issued;
        Statistic<uint64_t>* stat_loads_issued;
        Statistic<uint64_t>* stat_fences_issued;
        Statistic<uint64_t>* stat_stores_executed;
        Statistic<uint64_t>* stat_loads_executed;
        Statistic<uint64_t>* stat_fences_executed;
        Statistic<uint64_t>* stat_split_stores;
        Statistic<uint64_t>* stat_split_loads;
        Statistic<uint64_t>* stat_stored_bytes;
        Statistic<uint64_t>* stat_loaded_bytes;
        Statistic<uint64_t>* stat_mem_loads_speculated;
        Statistic<uint64_t>* stat_mem_loads_forwarded;
        Statistic<uint64_t>* stat_mem_violations;
        Statistic<uint64_t>* stat_mem_replays;
        Statistic<uint64_t>* stat_mem_predictor_holds;
};

} // namespace SST
}
#endif
