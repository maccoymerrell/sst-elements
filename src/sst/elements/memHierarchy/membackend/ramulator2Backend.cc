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


#include <sst_config.h>

#include <fstream>
#include <iostream>
#include "sst/elements/memHierarchy/util.h"
#include "membackend/ramulator2Backend.h"
#include "ramulator/base/config.h"

using namespace SST;
using namespace SST::MemHierarchy;


ramulator2Memory::ramulator2Memory(ComponentId_t id, Params &params) :
    SimpleMemBackend(id, params)
{
    config_path = params.find<std::string>("configFile",
                                            NO_STRING_DEFINED);
    if (config_path == NO_STRING_DEFINED) {
        output->fatal(CALL_INFO, -1, "Ramulator2 Backend must define a 'configFile' file parameter\n");
    }

    stats_path = params.find<std::string>("statsFile", "");

    // Our ramulator2 fork returns its own ConfigNode rather than a YAML::Node,
    // and parse_config_file takes only the path. See NMFC-Rev/README.md: we keep
    // one ramulator across both simulators so the memory device is identical by
    // construction, not by assertion, and adapt this backend to it.
    Ramulator::ConfigNode config = Ramulator::Config::parse_config_file(config_path);
    ramulator2_frontend = Ramulator::Factory::create_frontend(config);
    ramulator2_memorysystem = Ramulator::Factory::create_memory_system(config);

    ramulator2_frontend->connect_memory_system(ramulator2_memorysystem);
    ramulator2_memorysystem->connect_frontend(ramulator2_frontend);

    output->output(CALL_INFO, "Instantiated Ramulator2 from config file %s\n", config_path.c_str());
}

bool ramulator2Memory::issueRequest(ReqId reqId, Addr addr, bool isWrite, unsigned numBytes){
    bool enqueue_success = false;

    if (isWrite) {
        // Our fork takes the request size: a cache block wider than one DRAM
        // transaction must be split, and upstream's 4-argument form silently
        // ignored size and modelled every request as a single transaction.
        enqueue_success = ramulator2_frontend->receive_external_requests(1, addr, 0,
            [this](Ramulator::Request& req) {}, numBytes);
        if (enqueue_success) {
            writes.insert(reqId);
        }
    } else {
        enqueue_success = ramulator2_frontend->receive_external_requests(0, addr, 0,
            [this](Ramulator::Request& req) {
                output->debug(_L10_, "Ramulator2Backend: Read callback\n");
                std::deque<ReqId> &reqs = dramReqs[req.addr];

                if (reqs.empty())
                    output->fatal(CALL_INFO, -1, "Ramulator2Backend: Error - ramulator2Done called but dramReqs[addr] is empty. Addr: %" PRIx64 "\n", (Addr)req.addr);

                ReqId memreq = reqs.front();
                reqs.pop_front();
                if(0 == reqs.size())
                    dramReqs.erase(req.addr);

                handleMemResponse(memreq);
        }, numBytes);
        if (enqueue_success) {
            if (dramReqs.find(addr) != dramReqs.end()) dramReqs[addr].push_back(reqId);
            else {
                std::deque<ReqId> reqs;
                reqs.push_back(reqId);
                dramReqs.insert(std::make_pair(addr,reqs));
            }
        }
    }
    output->debug(_L10_, "Ramulator2Backend: enqueue %s\n", enqueue_success ? "successful" : "unsuccessful");

    return enqueue_success;
}

bool ramulator2Memory::clock(Cycle_t cycle){
#ifdef __SST_DEBUG_OUTPUT__
    output->debug(_L10_, "Ramulator2Backend: Ticking memory system.\n");
#endif
    ramulator2_frontend->tick();
    // Ack writes since ramulator won't
    while (!writes.empty()) {
        handleMemResponse(*writes.begin());
        writes.erase(writes.begin());
    }
    return false;
}

void ramulator2Memory::finish(){
    ramulator2_frontend->finalize();
    ramulator2_memorysystem->finalize();

    // Derived statistics -- spreads, peaks, means -- are computed in each
    // component's update_stats(), not in finalize(), and nothing calls it on
    // its own. Skipping this does not fail: the raw counters print correctly
    // and everything computed from them prints as zero, which reads exactly
    // like a plugin that saw no traffic. Ramulator's own python bindings run
    // finalize on both, then update_stats_recursive on both, then read.
    ramulator2_frontend->update_stats_recursive();
    ramulator2_memorysystem->update_stats_recursive();

    // Ramulator keeps its own statistics -- bank balance, command gaps, queue
    // occupancy -- and they are not SST statistics, so nothing else prints
    // them. ChampSim calls print_stats itself; without this the numbers exist
    // and are simply never emitted, which is worse than not collecting them.
    if ( stats_path.empty() ) {
        ramulator2_memorysystem->print_stats(std::cout);
    } else {
        std::ofstream out(stats_path);
        if ( out ) ramulator2_memorysystem->print_stats(out);
        else output->fatal(CALL_INFO, -1, "ramulator2: cannot write statsFile \"%s\"\n", stats_path.c_str());
    }
}
