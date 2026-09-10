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

#ifndef _H_VANADIS_LSQ_BASE
#define _H_VANADIS_LSQ_BASE

#include <sst/core/output.h>
#include <sst/core/subcomponent.h>

#include "inst/regfile.h"
#include "inst/vfence.h"
#include "inst/vload.h"
#include "inst/vstore.h"

#include <cassert>
#include <cinttypes>
#include <cstdint>
#include <vector>
#include <queue>

#define VANADIS_DBG_LSQ_STORE_FLG 0 // (1<<0)
#define VANADIS_DBG_LSQ_LOAD_FLG  0 //(1<<1)

namespace SST {
namespace Vanadis {

class VanadisLoadStoreQueue : public SST::SubComponent {

public:
    SST_ELI_REGISTER_SUBCOMPONENT_API(SST::Vanadis::VanadisLoadStoreQueue, int, int)

    SST_ELI_DOCUMENT_PARAMS({ "verbose", "Set the verbosity of output for the LSQ", "0" },
                            { "verboseMask", "Mask bits for masking output", "-1" },
                            { "dbgInsAddrs", "Comma-separated list of instruction addresses to debug", ""},
                            { "dbgAddrs", "Comma-separated list of addresses to debug", ""},
                            { "lsq_speculate", "1 lets a load issue past an older store whose address is not yet known, forward from an older store, and be replayed when an older store later resolves onto its bytes. 0 is the strictly in-order queue.", "1"},
                            { "mem_dep_speculation", "Alias of lsq_speculate.", "1"},
                            { "lsq_forward", "1 lets a load be answered out of an older store's bytes; 0 makes it wait for the store to reach memory", "1"},
                            { "mem_dep_predictor", "Which memory-dependence predictor holds a load back from speculating: counter, store_pc, none, or hold", "counter"},
                            { "mdp_entries", "Two-bit counters in the counter predictor, rounded up to a power of two", "4096"},
                            { "mdp_store_entries", "Entries in the store-address predictor, rounded up to a power of two", "4096"},
                            { "mdp_counter_decay", "When the counter predictor counts down: retire (every clean retire) or speculated (only when the load actually went past an unknown store)", "retire"},
            )

    /*
     * Constructor takes two additional parameters
     * coreid - an integer specifying the core ID for the core that loaded this LSQ
     *          - Each vanadis core should have a unique and sequential ID. E.g., for one core the ID=0, for two cores, one has ID=0 and one has ID=1
     * hwthreads - the total number of hardware threads supported by the core. The LSQ will use this to appropriately partition queues and detect
     *              ordering violations
     */
    VanadisLoadStoreQueue(ComponentId_t id, Params& params, int coreid, int hwthreads) : SubComponent(id) {

        uint32_t verbosity = params.find<uint32_t>("verbose");
        uint32_t mask = params.find<uint32_t>("verboseMask",-1);
        std::string prefix = "[lsq " + getName() + " !t]: ";
        output = new SST::Output(prefix, verbosity, mask, SST::Output::STDOUT);

        setDbgInsAddrs( params.find<std::string>("dbgInsAddrs", "") );
        setDbgAddrs( params.find<std::string>("dbgAddrs", "") );

        core_id = coreid;
        hw_threads = hwthreads;
        registerFiles = nullptr;
    }

    virtual ~VanadisLoadStoreQueue() { delete output; }

    void setRegisterFiles(std::vector<VanadisRegisterFile*>* reg_f) {
        output->verbose(CALL_INFO, 8, 0, "Setting register files (%" PRIu32 " register files in set)\n",
                        (uint32_t)reg_f->size());
        assert(reg_f != nullptr);

        registerFiles = reg_f;
    }
    // ---- THE WORK COUNTER --------------------------------------------------
    //
    // A workload counts units of its own useful output -- a vertex settled, one
    // sum performed, one insertion or lookup performed -- in one 64-bit global,
    // `nmfc_work`, and both builds of the workload increment it at the same
    // unit (src/nmfc/test/nmfc_work.h). That count is the coordinate a sampled
    // window is placed on when two builds are to be compared, because equal
    // instruction counts do not cover equal work: the offloaded build executes
    // instructions the pure-host build does not have.
    //
    // This core reads the counter the only way a core can: it watches the
    // program's committed stores to that address. A store issues from this
    // queue only at the head of the reorder buffer, so a store this queue sends
    // is a store the program really makes, in program order -- which is what
    // makes the published value the architectural value of the counter and not
    // a speculated one.
    //
    // The shadow is eight bytes wide whatever the program's counter is: a
    // narrower counter writes its own bytes and leaves the rest at the starting
    // value, so a 32-bit counter reads back as a 32-bit value. `start` is what
    // the counter already held when this run began, which for a run resumed
    // from a whole-program image is the value the image was captured with and
    // for a run started at the program's entry point is zero.
    void setWorkCounter(uint64_t addr, uint64_t width, uint64_t start)
    {
        work_addr_  = addr;
        work_width_ = (width == 0 || width > 8) ? 8 : width;
        work_watch_ = (addr != 0);
        work_value_ = start;
        for ( unsigned i = 0; i < 8; i++ ) { work_bytes_[i] = uint8_t(start >> (8 * i)); }
    }

    /// The work counter's value as the program's committed stores have left it.
    uint64_t workCounter() const { return work_value_; }
    /// Whether this queue is watching a counter at all.
    bool watchingWork() const { return work_watch_; }

    void setCoreId( int core ) { core_id = core; }
    int getCoreId( ) { return core_id; }

    void setHWThreads( int threads ) { hw_threads = threads; }
    int getHWThreads( ) { return hw_threads; }

    virtual bool storeFull() = 0;
    virtual bool loadFull() = 0;
    virtual bool storeBufferFull() = 0;

    virtual size_t storeSize() = 0;
    virtual size_t loadSize() = 0;
    virtual size_t storeBufferSize() = 0;

    virtual void push(VanadisStoreInstruction* store_me) = 0;
    virtual void push(VanadisLoadInstruction* load_me) = 0;
    virtual void push(VanadisFenceInstruction* fence) = 0;

    // ---- A SPECULATIVE QUEUE'S FOUR EXTRA OBLIGATIONS ----------------------
    //
    // A queue that lets a load issue past an older store needs the core to tell
    // it three things the in-order queue never needed, and needs the core to
    // ask it one.
    //
    // speculative()  Whether this queue reorders memory operations at all. The
    //                core asks so that it knows whether it may hand the queue a
    //                memory instruction that is not the oldest one outstanding.
    //                A queue that answers false is driven exactly as before.
    //
    // reserve()      A slot is taken when the instruction is RENAMED, not when
    //                it issues, because a load can only be told to wait for an
    //                older store whose address is unknown if that store already
    //                holds a slot saying it is older. The core refuses to
    //                rename a load when loadFull(), a store when storeFull().
    //
    // commit()       A load's slot is freed when the load RETIRES, not when its
    //                value arrives: until then an older store may still resolve
    //                onto its bytes and invalidate it.
    //
    // noteReplay()   The load at the head of the reorder buffer is about to be
    //                re-executed because an older store wrote bytes it had
    //                already read. Called before the repair, while the queue
    //                still holds the record of what happened.
    virtual bool speculative() const { return false; }

    // TRUE WHILE AN ORDERING INSTRUCTION IS IN THE QUEUE AND HAS NOT EXECUTED.
    // A fence, a load-linked, a store-conditional and a locked access are the
    // instructions the machine's synchronisation is written in terms of, and
    // nothing younger may be handed to a speculative queue while one of them is
    // outstanding -- the queue would resolve it straight away and it would pass
    // the fence. The in-order queue needs no such answer: its single queue puts
    // everything behind the fence by construction.
    virtual bool orderedPending(const uint32_t thread) { return false; }
    virtual void reserve(VanadisInstruction* ins) {}
    virtual void commit(VanadisInstruction* ins) {}
    virtual void noteReplay(VanadisInstruction* ins) {}
    virtual void resetPredictors(const uint32_t thread) {}

    // ---- ADDRESS GENERATION, MEMORY PORTS AND TRANSLATION ------------------
    //
    // A memory instruction does not step straight from the scheduler into this
    // queue in a real core: an address-generation unit computes its effective
    // address first, which occupies one of a small number of such units for a
    // cycle, and the operation then enters the memory pipeline through a load
    // port or a store port, of which there are separately a few. A queue that
    // models none of this answers false to aguEnabled() and the core hands it
    // instructions exactly as it always did.
    //
    // aguEnabled()       whether this queue models address generation at all.
    // aguAvailable()     an address-generation unit AND a port of the right
    //                    kind are free this cycle.
    // aguPipeEmpty()     nothing is between address generation and the queue.
    //                    An ordering instruction -- a fence, a load-linked, a
    //                    store-conditional, a locked access -- is handed over
    //                    only when this is true, and nothing is handed over
    //                    while an ordering instruction is in there, which is
    //                    what keeps a one-cycle pipeline from reordering the
    //                    instructions whose whole purpose is their order.
    // pushViaAGU()       hand the instruction to address generation. It reaches
    //                    the queue proper a configured number of cycles later,
    //                    and not until its translation is in hand.
    virtual bool aguEnabled() const { return false; }
    virtual bool aguAvailable(const bool is_store) const { return true; }
    virtual bool aguPipeEmpty() const { return true; }
    virtual bool aguOrderedInPipe() const { return false; }
    virtual void pushViaAGU(VanadisInstruction* ins, const uint64_t cycle) {}

    /// A thread's window has been thrown away and its instructions are about to
    /// be deleted, so nothing in address generation may still point at one.
    /// Called wherever the reorder buffer is emptied, including the paths that
    /// do not otherwise touch this queue.
    virtual void dropAGUByThreadID(const uint32_t thread) {}

    /// The host's translation path, or nullptr when this queue has none. It is
    /// shared: the instruction side asks the same unit, because the second-level
    /// buffer and the walkers are shared structures on every reference core.

    virtual void tick(uint64_t cycle) = 0;
    virtual void clearLSQByThreadID(const uint32_t thread) = 0;

    virtual void init(unsigned int phase) = 0;

    virtual void printStatus(SST::Output& output) {}

protected:

    /// TRUE WHEN A STORE TOUCHES THE WORK COUNTER. Called on the store path for
    /// every committed store, so it is one comparison against a constant.
    bool storeTouchesWork(uint64_t address, uint64_t width) const
    {
        return work_watch_ && (address < work_addr_ + work_width_) && (work_addr_ < address + width);
    }

    /// ONE COMMITTED STORE'S BYTES, merged into the shadow of the counter.
    ///
    /// A store may be narrower than the counter, may be wider, and may be
    /// offset inside it, so the overlap is taken byte by byte rather than
    /// assuming a store writes the whole word. The bytes are little-endian,
    /// which is this machine's order.
    void noteWorkStore(uint64_t address, const uint8_t* bytes, uint64_t width)
    {
        for ( uint64_t i = 0; i < width; i++ ) {
            const uint64_t a = address + i;
            if ( a < work_addr_ || a >= work_addr_ + work_width_ ) { continue; }
            work_bytes_[a - work_addr_] = bytes[i];
        }
        uint64_t v = 0;
        for ( unsigned i = 0; i < 8; i++ ) { v |= uint64_t(work_bytes_[i]) << (8 * i); }
        work_value_ = v;
    }

    uint64_t work_addr_  = 0;
    uint64_t work_width_ = 8;
    uint64_t work_value_ = 0;
    bool     work_watch_ = false;
    uint8_t  work_bytes_[8] = {0};

    void setDbgInsAddrs( std::string addrs ) {

        while ( ! addrs.empty() ) {
            printf("%s() %s\n",__func__,addrs.c_str());
            auto pos = addrs.find(',');
            std::string addr;
            if ( pos == std::string::npos ) {
                addr = addrs;
                addrs.clear();
            } else  {
                addr = addrs.substr(0,pos);
                addrs = addrs.substr(pos+1);
            }
            m_dbgInsAddrs.push_back(  strtol( addr.c_str(), NULL , 16 ) );
        }
    }


    bool isDbgInsAddr( uint64_t addr ) {
        for ( auto& it : m_dbgInsAddrs ) {
            if ( it == addr ) return true;
        }
        return false;
    }

    void setDbgAddrs( std::string addrs ) {

        while ( ! addrs.empty() ) {
            printf("%s() %s\n",__func__,addrs.c_str());
            auto pos = addrs.find(',');
            std::string addr;
            if ( pos == std::string::npos ) {
                addr = addrs;
                addrs.clear();
            } else  {
                addr = addrs.substr(0,pos);
                addrs = addrs.substr(pos+1);
            }
            m_dbgAddrs.push_back(  strtol( addr.c_str(), NULL , 16 ) );
        }
    }


    bool isDbgAddr( uint64_t addr ) {
        for ( auto& it : m_dbgAddrs ) {
            if ( it == addr ) return true;
        }
        return false;
    }

    std::deque<uint64_t> m_dbgInsAddrs;
    std::deque<uint64_t> m_dbgAddrs;
    int core_id;
    int hw_threads;
    std::vector<VanadisRegisterFile*>* registerFiles;
    SST::Output* output;
};

} // namespace Vanadis
} // namespace SST

#endif
