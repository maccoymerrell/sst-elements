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

#ifndef _H_VANADIS_DECODER
#define _H_VANADIS_DECODER

#include "datastruct/cqueue.h"
#include "decoder/visaopts.h"
#include "inst/fpregmode.h"
#include "inst/isatable.h"
#include "inst/vinst.h"
#include "lsq/vlsq.h"
#include "os/vcpuos.h"
#include "vbranch/vbranchbasic.h"
#include "vbranch/vbranchunit.h"
#include "velf/velfinfo.h"
#include "vfdip.h"
#include "vinsloader.h"
#include "vfpflags.h"


#include <cinttypes>
#include <cstdint>
#include <sst/core/interfaces/stdMem.h>
#include <sst/core/subcomponent.h>

#define VANADIS_DECODER_ELI_STATISTICS                                                                \
    { "uop_cache_hit", "Count number of times the instruction micro-op cache is hit", "hits", 1 },    \
        { "predecode_cache_hit",                                                                      \
          "Count number of times the predecode cache is hit when decoding an "                        \
          "instruction",                                                                              \
          "hits", 1 },                                                                                \
        { "predecode_cache_miss",                                                                     \
          "Count number of times the predecode cache misses, this forces a load "                     \
          "from the instruction cache interface",                                                     \
          "misses", 1 },                                                                              \
        { "decode_faults",                                                                            \
          "Count number of times decode operation fails to generate valid "                           \
          "micro-ops",                                                                                \
          "uops", 1 },                                                                                \
        { "ins_bytes_loaded", "Count the number of bytes loaded for decode operations", "bytes", 1 }, \
        { "uop_delayed_rob_full", "Number of times a micro-op cannot be added to the ROB because it is full.", "cycles", 1 }, \
        { "fetch_stall_icache", "Decode ticks that produced nothing because the instruction bytes were not there yet", "cycles", 1 }, \
        { "icache_demand_fetches", "Demand instruction-cache line requests issued by this decoder", "lines", 1 }, \
        { "fdip_ftq_occupancy", "Fetch target queue occupancy, sampled every cycle", "entries", 1 },  \
        { "fdip_ftq_full_cycles", "Cycles the fetch target queue was full", "cycles", 1 },            \
        { "fdip_ftq_empty_cycles", "Cycles the fetch target queue was empty", "cycles", 1 },          \
        { "fdip_blocks_produced", "Fetch blocks the run-ahead predictor put in the FTQ", "blocks", 1 }, \
        { "fdip_runahead_blocked", "Cycles the run-ahead could not continue because an indirect target was unknown", "cycles", 1 }, \
        { "fdip_ftq_flushes", "Times the fetch target queue was thrown away", "flushes", 1 },         \
        { "fdip_ftq_flushed_entries", "FTQ entries discarded by those flushes", "entries", 1 },       \
        { "frontend_override", "Times the decode stage overrode the fetch stage's prediction", "branches", 1 }, \
        { "frontend_bubble", "Cycles decode produced nothing while the front end re-steered", "cycles", 1 }, \
        { "frontend_btb_miss", "Branches the decode stage met that the fetch stage had not predicted", "branches", 1 }, \
        { "fdip_btb_bubbles", "Prediction bubbles charged by second-level buffer hits", "cycles", 1 },  \
        { "fdip_fetch_branches", "Branches the fetch stage predicted", "branches", 1 },               \
        { "fdip_decode_hit", "Branches the decode stage found a fetch-stage prediction for", "branches", 1 }, \
        { "fdip_decode_miss", "Branches the decode stage met with no fetch-stage prediction", "branches", 1 }, \
        { "fdip_pf_issued", "Instruction prefetches issued", "prefetches", 1 },                       \
        { "fdip_pf_useful", "Prefetched lines a demand fetch then asked for", "prefetches", 1 },      \
        { "fdip_pf_late", "Of those, the ones still in flight when the demand asked", "prefetches", 1 }, \
        { "fdip_pf_useless", "Prefetched lines dropped from the record with no demand for them", "prefetches", 1 }, \
        { "fdip_pf_filtered", "FTQ lines not prefetched because the engine had recently seen them", "lines", 1 }, \
        { "fdip_pf_in_line_buffer", "FTQ lines already in the core's line buffer", "lines", 1 },      \
        { "fdip_pf_demand_pending", "FTQ lines a demand fetch was already in flight for", "lines", 1 }, \
        { "fdip_pf_refused", "Cycles a prefetch was held back by the outstanding-prefetch limit", "events", 1 }, \
        { "fdip_pf_dropped_response", "Prefetch responses received and discarded", "responses", 1 },  \
    {                                                                                                 \
        "uops_generated",                                                                             \
            "Count number of micro-ops generated by decoder that are transfered to "                  \
            "the pipeline for execution",                                                             \
            "uops", 1                                                                                 \
    }

namespace SST {
namespace Vanadis {

class VanadisDecoder : public SST::SubComponent
{
public:
    SST_ELI_REGISTER_SUBCOMPONENT_API(SST::Vanadis::VanadisDecoder, SST::Output*)

    SST_ELI_DOCUMENT_SUBCOMPONENT_SLOTS({ "os_handler", "Handler for SYSCALL instructions",
                                          "SST::Vanadis::VanadisCPUOSHandler" },
                                        { "branch_unit", "Branch prediction unit", "SST::Vanadis::VanadisBranchUnit" })

    SST_ELI_DOCUMENT_PARAMS(
            //{ "decode_q_len", "Number of entries in the decoded, but pending issue queue" },
                            { "icache_line_width", "Number of bytes in an icache line", "64"},
                            { "uop_cache_entries",
                              "Number of instructions to cache in the micro-op cache (this is full "
                              "instructions, not microops but usually 1:1 ratio", "128" },
                            { "predecode_cache_entries",
                              "Number of cache lines to store in the local L0 cache for instructions "
                              "pending decoding.", "4" },
                            { "loader_mode",
                              "Operation of the loader, 0 = LRU (more accurate), 1 = INFINITE cache (faster simulation)", "0"},
                            { "fdip_enable", "Fetch-directed instruction prefetching: the predictor runs ahead of the instruction cache into a fetch target queue and the lines it names are prefetched", "1" },
                            { "fdip_ftq_entries", "Fetch target queue depth, in fetch blocks", "32" },
                            { "fdip_blocks_per_cycle", "Fetch blocks the run-ahead predictor produces per cycle", "2" },
                            { "fdip_prefetch_per_cycle", "Instruction prefetches issued per cycle", "2" },
                            { "fdip_max_outstanding", "Instruction prefetches allowed in flight at once", "8" },
                            { "frontend_override_bubble", "Cycles lost when the decode stage overrides the fetch stage's prediction", "3" },
                            { "fdip_l2_bubble", "Prediction bubbles charged when the second-level branch target buffer answers", "1" },
                            { "fdip_filter_sets", "Sets in the record of lines recently sent to the L1I", "64" },
                            { "fdip_filter_ways", "Ways in that record", "8" })

    SST_ELI_DOCUMENT_STATISTICS(
				VANADIS_DECODER_ELI_STATISTICS
				)

    VanadisDecoder(ComponentId_t id, Params& params, SST::Output* output) : SubComponent(id)
    {
        ip      = 0;
        tls_ptr = 0;

        thread_rob = nullptr;
		  fpflags = nullptr;

        icache_line_width = params.find<uint64_t>("icache_line_width", 64);

        const size_t uop_cache_size          = params.find<size_t>("uop_cache_entries", 128);
        const size_t predecode_cache_entries = params.find<size_t>("predecode_cache_entries", 4);

        ins_loader = new VanadisInstructionLoader(uop_cache_size, predecode_cache_entries, icache_line_width, output);

        const uint32_t loader_mode = params.find<uint32_t>("loader_mode", 0);
        switch(loader_mode) {
        case 0:
            ins_loader->setLoaderMode(VanadisInstructionLoaderMode::LRU_CACHE_MODE);
            break;
        case 1:
            ins_loader->setLoaderMode(VanadisInstructionLoaderMode::INFINITE_CACHE_MODE);
            break;
        default:
            ins_loader->setLoaderMode(VanadisInstructionLoaderMode::LRU_CACHE_MODE);
            break;
        }

        branch_predictor = loadUserSubComponent<SST::Vanadis::VanadisBranchUnit>("branch_unit");
        os_handler       = loadUserSubComponent<SST::Vanadis::VanadisCPUOSHandler>("os_handler");

        if ( !branch_predictor ) {
            fatal(CALL_INFO, -1, "Error: Unable to load a branch predictor from the decoder's 'branch_unit' subcomponent slot\n");
        }

        if ( !os_handler ) {
            fatal(CALL_INFO, -1, "Error: Unable to load an OS handler from the decoder's 'os_handler' subcomponent slot\n");
        }

        hw_thr = 0;

        os_handler->setThreadLocalStoragePointer(&tls_ptr);

        canIssueStores = true;
        canIssueLoads  = true;

        output_ = output;

        stat_uop_hit          = registerStatistic<uint64_t>("uop_cache_hit", "1");
        stat_predecode_hit    = registerStatistic<uint64_t>("predecode_cache_hit", "1");
        stat_predecode_miss   = registerStatistic<uint64_t>("predecode_cache_miss", "1");
        stat_uop_generated    = registerStatistic<uint64_t>("uops_generated", "1");
        stat_decode_fault     = registerStatistic<uint64_t>("decode_faults", "1");
        stat_ins_bytes_loaded = registerStatistic<uint64_t>("ins_bytes_loaded", "1");
        stat_uop_delayed_rob_full = registerStatistic<uint64_t>("uop_delayed_rob_full", "1");
        stat_fetch_stall_icache   = registerStatistic<uint64_t>("fetch_stall_icache", "1");
        stat_icache_demand        = registerStatistic<uint64_t>("icache_demand_fetches", "1");

        // THE COST OF THE SECOND STAGE. AMD's Zen 4 optimisation guide gives
        // the published figure for a later predictor overriding the one that
        // steered fetch: "The L2 BTB has 7680 entries and creates three
        // prediction bubbles if its prediction differs from that of the L1
        // BTB" (publication 57647, section 2.8.1.2).
        frontend_override_bubble = params.find<uint64_t>("frontend_override_bubble", 3);
        frontend_resteer_until   = 0;

        stat_frontend_override = registerStatistic<uint64_t>("frontend_override", "1");
        stat_frontend_bubble   = registerStatistic<uint64_t>("frontend_bubble", "1");
        stat_frontend_btb_miss = registerStatistic<uint64_t>("frontend_btb_miss", "1");

        // FETCH-DIRECTED INSTRUCTION PREFETCHING (vfdip.h). Default on.
        VanadisFDIP::Config fdip_cfg;
        fdip_cfg.enable             = params.find<bool>("fdip_enable", true);
        fdip_cfg.ftq_entries        = params.find<uint32_t>("fdip_ftq_entries", 32);
        fdip_cfg.blocks_per_cycle   = params.find<uint32_t>("fdip_blocks_per_cycle", 2);
        fdip_cfg.prefetch_per_cycle = params.find<uint32_t>("fdip_prefetch_per_cycle", 2);
        fdip_cfg.max_outstanding    = params.find<uint32_t>("fdip_max_outstanding", 8);
        fdip_cfg.l2_bubble          = params.find<uint32_t>("fdip_l2_bubble", 1);
        fdip_cfg.filter_sets        = params.find<uint32_t>("fdip_filter_sets", 64);
        fdip_cfg.filter_ways        = params.find<uint32_t>("fdip_filter_ways", 8);
        fdip_cfg.line_width         = icache_line_width;

        fdip = new VanadisFDIP(fdip_cfg, ins_loader, branch_predictor, output);

        VanadisFDIPStats fdip_stats;
        fdip_stats.ftq_occupancy       = registerStatistic<uint64_t>("fdip_ftq_occupancy", "1");
        fdip_stats.ftq_full_cycles     = registerStatistic<uint64_t>("fdip_ftq_full_cycles", "1");
        fdip_stats.ftq_empty_cycles    = registerStatistic<uint64_t>("fdip_ftq_empty_cycles", "1");
        fdip_stats.blocks_produced     = registerStatistic<uint64_t>("fdip_blocks_produced", "1");
        fdip_stats.runahead_blocked    = registerStatistic<uint64_t>("fdip_runahead_blocked", "1");
        fdip_stats.ftq_flushes         = registerStatistic<uint64_t>("fdip_ftq_flushes", "1");
        fdip_stats.ftq_flushed_entries = registerStatistic<uint64_t>("fdip_ftq_flushed_entries", "1");
        fdip_stats.btb_bubbles         = registerStatistic<uint64_t>("fdip_btb_bubbles", "1");
        fdip_stats.fetch_branches      = registerStatistic<uint64_t>("fdip_fetch_branches", "1");
        fdip_stats.decode_hit          = registerStatistic<uint64_t>("fdip_decode_hit", "1");
        fdip_stats.decode_miss         = registerStatistic<uint64_t>("fdip_decode_miss", "1");
        fdip_stats.pf_issued           = registerStatistic<uint64_t>("fdip_pf_issued", "1");
        fdip_stats.pf_useful           = registerStatistic<uint64_t>("fdip_pf_useful", "1");
        fdip_stats.pf_late             = registerStatistic<uint64_t>("fdip_pf_late", "1");
        fdip_stats.pf_useless          = registerStatistic<uint64_t>("fdip_pf_useless", "1");
        fdip_stats.pf_filtered         = registerStatistic<uint64_t>("fdip_pf_filtered", "1");
        fdip_stats.pf_in_line_buffer   = registerStatistic<uint64_t>("fdip_pf_in_line_buffer", "1");
        fdip_stats.pf_demand_pending   = registerStatistic<uint64_t>("fdip_pf_demand_pending", "1");
        fdip_stats.pf_refused          = registerStatistic<uint64_t>("fdip_pf_refused", "1");
        fdip_stats.pf_dropped_response = registerStatistic<uint64_t>("fdip_pf_dropped_response", "1");
        fdip->setStatistics(fdip_stats);
    }

    virtual ~VanadisDecoder()
    {
        delete fdip;
        delete ins_loader;
        delete os_handler;
        delete branch_predictor;
    }

    virtual void markLoadFencing() { canIssueLoads = false; }

    virtual void markStoreFencing() { canIssueStores = false; }

    virtual void clearLoadFencing() { canIssueLoads = true; }

    virtual void clearStoreFencing() { canIssueStores = true; }

    virtual void clearFencing()
    {
        clearLoadFencing();
        clearStoreFencing();
    }

    virtual void markFencing()
    {
        markLoadFencing();
        markStoreFencing();
    }

    void setInsCacheLineWidth(const uint64_t ic_width)
    {
        icache_line_width = ic_width;
        ins_loader->setCacheLineWidth(ic_width);
    }

    void setFPFlags(VanadisFloatingPointFlags* new_fpflags) {
		fpflags = new_fpflags;
	 }

    bool acceptCacheResponse( SST::Interfaces::StandardMem::Request* req )
    {
        return ins_loader->acceptResponse(req);
    }

    uint64_t getInsCacheLineWidth() const { return icache_line_width; }

    virtual VanadisFPRegisterMode getFPRegisterMode() const = 0;

    virtual const char*                  getISAName() const                        = 0;
    virtual uint16_t                     countISAIntReg() const                    = 0;
    virtual uint16_t                     countISAFPReg() const                     = 0;
    virtual bool                         tick( uint64_t cycle ) = 0;
    virtual const VanadisDecoderOptions* getDecoderOptions() const                 = 0;

    uint64_t getInstructionPointer() const { return ip; }

    void setInstructionPointer(const uint64_t newIP)
    {
        ip = newIP;

        // Do we need to clear here or not?
    }

    virtual void setStackPointer( VanadisISATable* isa_tbl, VanadisRegisterFile* regFile, const uint64_t stack_start_address ) {assert(0);}
    virtual void setThreadPointer( VanadisISATable* isa_tbl, VanadisRegisterFile* regFile, const uint64_t stack_start_address ) {}
    virtual void setArg1Register( VanadisISATable* isa_tbl, VanadisRegisterFile* regFile, const uint64_t value ) {assert(0);}
    virtual void setFuncPointer( VanadisISATable* isa_tbl, VanadisRegisterFile* regFile, const uint64_t value ) {}
    virtual void setReturnRegister( VanadisISATable* isa_tbl, VanadisRegisterFile* regFile, const uint64_t value ) {assert(0);}
    virtual void setSuccessRegister( VanadisISATable* isa_tbl, VanadisRegisterFile* regFile, const uint64_t value ) {}

    void setInstructionPointerAfterMisspeculate(const uint64_t newIP)
    {
        ip = newIP;

        // THE FETCH TARGET QUEUE DESCRIBES A PATH THE CORE IS NOT TAKING.
        // Every redirect reaches this function -- a branch misprediction, a
        // fault, a system call's resume, a thread start -- so this is the one
        // place the queue has to be discarded and the run-ahead's own history
        // and return stack put back to the architected copies.
        fdip->flush(newIP);
        frontend_resteer_until = 0;

        output_->verbose(CALL_INFO, 16, 0, "[decoder] -> clear decode-q and set new ip: 0x%" PRI_ADDR "\n", newIP);

        // Clear out the decode queue, need to restart
        // decoded_q->clear();

        clearDecoderAfterMisspeculate();
    }

    void setThreadLocalStoragePointer(uint64_t new_tls) { tls_ptr = new_tls; }

    uint64_t getThreadLocalStoragePointer() const { return tls_ptr; }
    uint64_t getCycleCount() const { return cycle_count; }

    // VanadisCircularQueue<VanadisInstruction*>* getDecodedQueue() { return
    // decoded_q; }

    virtual void setThreadROB(VanadisCircularQueue<VanadisInstruction*>* thr_rob) { thread_rob = thr_rob; }

    void     setCore(const uint32_t num ) { core = num; }
    uint32_t getCore() const { return core; }

    void     setHardwareThread(const uint32_t thr) { hw_thr = thr; }
    uint32_t getHardwareThread() const { return hw_thr; }

    VanadisInstructionLoader* getInstructionLoader() { return ins_loader; }
    VanadisBranchUnit*        getBranchPredictor() { return branch_predictor; }
    VanadisFDIP*              getFDIP() { return fdip; }

    // Once a cycle, from the core's fetch stage. The run-ahead predictor
    // produces fetch blocks into the FTQ and the prefetch engine issues the
    // lines they name.
    void fdipTick(const uint64_t cycle)
    {
        cycle_count = cycle;
        fdip->tick(cycle, ip);
    }

    virtual VanadisCPUOSHandler* getOSHandler() { return os_handler; }

protected:
    virtual void clearDecoderAfterMisspeculate() {};

    uint64_t ip;
    uint64_t icache_line_width;
    uint32_t hw_thr;
    uint32_t core;

    uint64_t tls_ptr;
    uint64_t cycle_count;

    bool                                       wantDelegatedLoad;
    VanadisCircularQueue<VanadisInstruction*>* thread_rob;

    // VanadisCircularQueue<VanadisInstruction*>* decoded_q;

    VanadisInstructionLoader* ins_loader;
    VanadisBranchUnit*        branch_predictor;
    VanadisCPUOSHandler*      os_handler;
	VanadisFloatingPointFlags* fpflags;

    bool canIssueStores;
    bool canIssueLoads;

    SST::Output* output_;

    Statistic<uint64_t>* stat_uop_hit;
    Statistic<uint64_t>* stat_uop_delayed_rob_full;
    Statistic<uint64_t>* stat_predecode_hit;
    Statistic<uint64_t>* stat_predecode_miss;
    Statistic<uint64_t>* stat_decode_fault;
    Statistic<uint64_t>* stat_uop_generated;
    Statistic<uint64_t>* stat_ins_bytes_loaded;
    Statistic<uint64_t>* stat_fetch_stall_icache;
    Statistic<uint64_t>* stat_icache_demand;
    Statistic<uint64_t>* stat_frontend_override;
    Statistic<uint64_t>* stat_frontend_bubble;
    Statistic<uint64_t>* stat_frontend_btb_miss;

    uint64_t frontend_override_bubble;
    uint64_t frontend_resteer_until;
    uint64_t frontend_bubble_cycle = 0;

    VanadisFDIP* fdip;
};


} // namespace Vanadis
} // namespace SST

#endif
