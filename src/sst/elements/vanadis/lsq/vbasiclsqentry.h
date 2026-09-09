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

#include <map>


#include <sst/core/interfaces/stdMem.h>

#include "inst/vinst.h"
#include "inst/vload.h"
#include "inst/vstore.h"
#include "inst/vfence.h"

using namespace SST::Interfaces;

namespace SST {
namespace Vanadis {

enum class VanadisBasicLoadStoreEntryOp {
    LOAD,
    STORE,
    FENCE
};

class VanadisBasicLoadStoreEntry {
public:
    VanadisBasicLoadStoreEntry(VanadisInstruction* the_ins) : ins(the_ins) {sw_thr=65536;}
    virtual ~VanadisBasicLoadStoreEntry() {}
    virtual VanadisBasicLoadStoreEntryOp getEntryOp() = 0;
    virtual VanadisInstruction* getInstruction() { return ins; }

    virtual bool isInstructionIssued() const { return ins->completedIssue(); }
    virtual bool isinstructionExecuted() const { return ins->completedExecution(); }

    uint32_t getHWThread() const { return ins->getHWThread(); }
    uint64_t getInstructionAddress() const { return ins->getInstructionAddress(); }

    uint32_t getSWThr() { return sw_thr; }
    void setSWThr(uint32_t thr) { sw_thr = thr; }
    void addThr(uint16_t thr) {sw_thrs.push_back(thr);}
    uint16_t getCoalescedSwThr(int i)
    {
        if(i>=sw_thrs.size())
        {
            return 0;
        }
        return sw_thrs[i];
    }
    uint16_t getNumCoalescedThreads()
    {
        return sw_thrs.size();
    }

protected:
    VanadisInstruction* ins;
    uint32_t sw_thr;
    std::vector<uint16_t> sw_thrs;

};

class VanadisBasicFenceEntry : public VanadisBasicLoadStoreEntry {
public:
    VanadisBasicFenceEntry(VanadisFenceInstruction* fence_ins) : VanadisBasicLoadStoreEntry(fence_ins) {}

    VanadisBasicLoadStoreEntryOp getEntryOp() override {
        return VanadisBasicLoadStoreEntryOp::FENCE;
    }

    VanadisFenceInstruction* getStoreInstruction() {
        return dynamic_cast<VanadisFenceInstruction*>(ins);
    }
};

class VanadisBasicStoreEntry : public VanadisBasicLoadStoreEntry {
public:
    VanadisBasicStoreEntry(VanadisStoreInstruction* store_ins) : VanadisBasicLoadStoreEntry(store_ins) {}

    VanadisBasicLoadStoreEntryOp getEntryOp() override {
        return VanadisBasicLoadStoreEntryOp::STORE;
    }

    VanadisStoreInstruction* getStoreInstruction() {
        return dynamic_cast<VanadisStoreInstruction*> (ins);
    }
};

// The widest store this queue will keep a copy of the bytes of, and therefore
// the widest one a younger load can be answered from without going to memory.
// Every store form the RISC-V decoders in this tree produce is eight bytes or
// narrower; the margin is so that a wider form added later is merely not
// forwarded from rather than mis-forwarded from.
#define VANADIS_LSQ_MAX_FWD_BYTES 16

class VanadisBasicStorePendingEntry : public VanadisBasicStoreEntry {
    public:
        // A store's address is known (RESOLVED) or it is not (RESERVED). The
        // distinction is the whole of what lets a younger load reason about it:
        // a load can be told to wait for a store whose address nobody knows yet
        // only if the store already occupies a queue slot that says it is older.
        enum StoreState { RESERVED, RESOLVED };

        VanadisBasicStorePendingEntry(VanadisStoreInstruction* store_ins, uint64_t addr, uint64_t width,
            VanadisStoreRegisterType valRegType, uint16_t valReg) :
            VanadisBasicStoreEntry(store_ins), storeAddress(addr), storeWidth(width),
            valueRegister(valReg), valueRegisterType(valRegType), dispatched(false),
            entry_age(0), state(RESOLVED), data_valid(false) {}

        // A SLOT TAKEN AT DISPATCH, before the address exists. `age` orders the
        // memory operations of one hardware thread in program order; smaller is
        // older.
        VanadisBasicStorePendingEntry(VanadisStoreInstruction* store_ins, uint64_t age) :
            VanadisBasicStoreEntry(store_ins), storeAddress(0), storeWidth(0),
            valueRegister(0), valueRegisterType(STORE_INT_REGISTER), dispatched(false),
            entry_age(age), state(RESERVED), data_valid(false) {}

        uint64_t getAge() const { return entry_age; }
        bool     isResolved() const { return RESOLVED == state; }

        // The address is computed and the value is copied out of the register
        // file in one step, because this core issues an instruction only when
        // every one of its operands is ready -- there is no separate
        // store-address operation to resolve on its own.
        void resolve(uint64_t addr, uint64_t width, VanadisStoreRegisterType valRegType, uint16_t valReg)
        {
            storeAddress      = addr;
            storeWidth        = width;
            valueRegister     = valReg;
            valueRegisterType = valRegType;
            state             = RESOLVED;
        }

        // The bytes this store will write, held so that a younger load which
        // this store fully covers can be answered from them.
        void     setForwardData(const uint8_t* bytes, uint64_t width)
        {
            if ( width > VANADIS_LSQ_MAX_FWD_BYTES ) { data_valid = false; return; }
            for ( uint64_t i = 0; i < width; ++i ) { fwd_data[i] = bytes[i]; }
            data_valid = true;
        }
        void           clearForwardData() { data_valid = false; }
        bool           canForward() const { return data_valid; }
        const uint8_t* forwardData() const { return fwd_data; }

        // True when this store writes every byte the load reads, which is the
        // only overlap that can be answered without merging bytes from more
        // than one place.
        bool fullyCovers(const uint64_t loadAddress, const uint64_t loadWidth) const {
            return (storeAddress <= loadAddress)
                && ((storeAddress + storeWidth) >= (loadAddress + loadWidth));
        }

        ~VanadisBasicStorePendingEntry() {
            requests.clear();
        }

        bool     isDispatched() const { return dispatched; }
        void     markDispatched() { dispatched = true; }

        uint64_t getStoreAddress() const { return storeAddress; }
        uint64_t getStoreWidth() const { return storeWidth; }
        uint16_t getValueRegister() const { return valueRegister; }

        size_t   countRequests() const { return requests.size(); }
        void     addRequest(StandardMem::Request::id_t req) { requests.push_back(req); }
        void     removeRequest(StandardMem::Request::id_t req) {
            for(auto req_itr = requests.begin(); req_itr != requests.end(); ) {
                if( (*req_itr) == req ) {
                    requests.erase(req_itr);
                    break;
                } else {
                    req_itr++;
                }
            }
        }
        bool     containsRequest(StandardMem::Request::id_t req) {
            bool found = false;

            for(auto req_itr = requests.begin(); req_itr != requests.end(); req_itr++) {
                if((*req_itr) == req) {
                    found = true;
                    break;
                }
            }

            return found;
        }

        bool    storeAddressOverlaps(const uint64_t loadAddress, const uint64_t loadWidth) const {
            bool overlaps = false;

            // Address Overlaps
            // An address overlaps with the store IF:
            // 1. the address being checked is within the range of the store being performed (overlaps fully/right side)
            // 2. the address + width being checked is within the range of the store being performed (overlaps left side)

            const auto store_end = storeAddress + storeWidth - 1;
            const auto load_end  = loadAddress + loadWidth - 1;

            /*
                There are five main cases to capture:
                    (S = start of store)
                    (L = start of load)

                Case 1: is overlap of load and store
                S-------|
                L-------|

                Case 2: load starts below a store and overlaps a small part at the beginning
                    S-------|
                L-------|

                Case 3: load starts within the region of the store and goes outside the range
                S-------|
                    L--------|

                Case 4: load is larger and covers the store
                    S--|
                L-------|

                Case 5: load is smaller and is covered by the store
                S--------|
                    L---|
            */

            // Load address is between the start and end of the store
            // captures cases 1, 3 and 5
            overlaps = ((loadAddress >= storeAddress) & (loadAddress <= (store_end)));

            // Load address is less than the start of the store but its end is within the store range
            // captures cases 1, 2 and 4
            overlaps |= ((loadAddress <= storeAddress) & (load_end >= storeAddress));

            return overlaps;
        }

        VanadisStoreRegisterType getValueRegisterType() const {
            return valueRegisterType;
        }

    protected:
        std::vector<StandardMem::Request::id_t> requests;
        uint64_t storeAddress;
        uint64_t storeWidth;
        uint16_t valueRegister;

        bool dispatched;
        VanadisStoreRegisterType valueRegisterType;

        uint64_t   entry_age;
        StoreState state;
        bool       data_valid;
        uint8_t    fwd_data[VANADIS_LSQ_MAX_FWD_BYTES] = {0};
    };


class VanadisBasicLoadEntry : public VanadisBasicLoadStoreEntry {
    public:
        VanadisBasicLoadEntry(VanadisLoadInstruction* load_ins) : VanadisBasicLoadStoreEntry(load_ins) {}

        VanadisBasicLoadStoreEntryOp getEntryOp() override {
            return VanadisBasicLoadStoreEntryOp::LOAD;
        }

        VanadisLoadInstruction* getLoadInstruction() {
            return dynamic_cast<VanadisLoadInstruction*>(ins);
        }
    };

class VanadisBasicLoadPendingEntry : public VanadisBasicLoadEntry {
    public:
        // RESERVED  slot taken at dispatch, address not computed yet
        // WAITING   address known, deciding what to do about older stores
        // INFLIGHT  a request is out at memory
        // DONE      the register has its value, from memory or from a store
        enum LoadState { RESERVED, WAITING, INFLIGHT, DONE };

        VanadisBasicLoadPendingEntry(VanadisLoadInstruction* load_ins, uint64_t address, uint64_t width) :
            VanadisBasicLoadEntry(load_ins), load_address(address), load_width(width),
            entry_age(0), state(INFLIGHT), speculated(false), held(false),
            fwd_from_age(0), violated(false), violating_store_pc(0) {}

        VanadisBasicLoadPendingEntry(VanadisLoadInstruction* load_ins, uint64_t age) :
            VanadisBasicLoadEntry(load_ins), load_address(0), load_width(0),
            entry_age(age), state(RESERVED), speculated(false), held(false),
            fwd_from_age(0), violated(false), violating_store_pc(0) {}

        uint64_t  getAge() const { return entry_age; }
        LoadState getState() const { return state; }
        void      setState(LoadState s) { state = s; }

        void resolve(uint64_t address, uint64_t width)
        {
            load_address = address;
            load_width   = width;
            state        = WAITING;
        }

        bool didSpeculate() const { return speculated; }
        void markSpeculated() { speculated = true; }

        // Counted once per load however many cycles it spends held, because the
        // statistic is a count of loads the predictor acted on and not of cycles.
        bool wasHeld() const { return held; }
        void markHeld() { held = true; }

        // The age of the store this load was answered from, or 0 for a load
        // answered by memory. A store younger than this one cannot have been
        // missed by this load: the load already saw a later value.
        uint64_t forwardedFrom() const { return fwd_from_age; }
        void     setForwardedFrom(uint64_t age) { fwd_from_age = age; }

        bool     hasViolated() const { return violated; }
        uint64_t violatingStorePC() const { return violating_store_pc; }
        void     markViolated(uint64_t store_pc) { violated = true; violating_store_pc = store_pc; }

        ~VanadisBasicLoadPendingEntry() {
            requests.clear();
        }

        void addRequest(StandardMem::Request::id_t req) {
            requests.push_back(req);
        }

        void addRequest(StandardMem::Request::id_t req, uint32_t sw_thr) {
            requests.push_back(req);
            setSWThr(sw_thr);
        }

        bool containsRequest(StandardMem::Request::id_t req) {
            bool found = false;

            for(auto req_itr = requests.begin(); req_itr != requests.end(); req_itr++) {
                if( req == (*req_itr) ) {
                    found = true;
                    break;
                }
            }

            return found;
        }



        void removeRequest(StandardMem::Request::id_t req) {
            for(auto req_itr = requests.begin(); req_itr != requests.end(); req_itr++) {
                if((*req_itr) == req) {
                    requests.erase(req_itr);
                    return;
                }
            }
        }

        uint64_t getLoadAddress() const {
            return load_address;
        }

        uint64_t getLoadWidth() const {
            return load_width;
        }

        size_t countRequests() const {
                return requests.size();
        }

        // identify what the req order is for this entry
        // in split-cache line loads we need to restore data into the register
        // in the correct order
        int identifySequence(StandardMem::Request::id_t req) const {
            int index = -1;

            for(int i = 0; i < requests.size(); ++i) {
                if(requests[i] == req) {
                    index = i;
                    break;
                }
            }

            return index;
        }
    protected:
        std::vector<StandardMem::Request::id_t> requests;
        // std::vector<uint32_t> request_swthr;
        // std::map<uint32_t, StandardMem::Request::id_t> requests;

        uint64_t load_address;
        uint64_t load_width;

        uint64_t  entry_age;
        LoadState state;
        bool      speculated;
        bool      held;
        uint64_t  fwd_from_age;
        bool      violated;
        uint64_t  violating_store_pc;
    };

}
}
