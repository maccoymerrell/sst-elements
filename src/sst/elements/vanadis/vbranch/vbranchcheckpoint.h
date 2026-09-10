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

#ifndef _H_VANADIS_BRANCH_CHECKPOINT
#define _H_VANADIS_BRANCH_CHECKPOINT

#include <cstddef>
#include <cstdint>
#include <vector>

namespace SST {
namespace Vanadis {

// What the decode already knows about a branch. The predictor needs it because
// the three kinds of target come from three different places: a direct branch
// carries its target in the instruction, a return's target is on the return
// address stack, and only a genuine indirect branch has to be looked up.
enum class VanadisBranchClass : uint8_t {
    CONDITIONAL   = 0, // BEQ/BNE/BLT/BGE/BLTU/BGEU, C.BEQZ, C.BNEZ
    DIRECT_JUMP   = 1, // JAL rd=x0, C.J
    DIRECT_CALL   = 2, // JAL rd=x1|x5, C.JAL
    INDIRECT_JUMP = 3, // JALR rd=x0 that is not a return, C.JR
    INDIRECT_CALL = 4, // JALR rd=x1|x5, C.JALR
    RETURN        = 5  // JALR rd=x0, rs1=x1|x5, imm=0, C.JR x1
};

inline bool
vanadisBranchIsConditional(const VanadisBranchClass c)
{
    return c == VanadisBranchClass::CONDITIONAL;
}

inline bool
vanadisBranchIsCall(const VanadisBranchClass c)
{
    return (c == VanadisBranchClass::DIRECT_CALL) || (c == VanadisBranchClass::INDIRECT_CALL);
}

inline bool
vanadisBranchIsIndirect(const VanadisBranchClass c)
{
    return (c == VanadisBranchClass::INDIRECT_JUMP) || (c == VanadisBranchClass::INDIRECT_CALL) ||
           (c == VanadisBranchClass::RETURN);
}

inline const char*
vanadisBranchClassName(const VanadisBranchClass c)
{
    switch ( c ) {
    case VanadisBranchClass::CONDITIONAL:
        return "conditional";
    case VanadisBranchClass::DIRECT_JUMP:
        return "direct-jump";
    case VanadisBranchClass::DIRECT_CALL:
        return "direct-call";
    case VanadisBranchClass::INDIRECT_JUMP:
        return "indirect-jump";
    case VanadisBranchClass::INDIRECT_CALL:
        return "indirect-call";
    case VanadisBranchClass::RETURN:
        return "return";
    }
    return "unknown";
}

// Opaque handle to the per-branch checkpoint. It travels with the branch
// micro-op through the reorder buffer: `slot` indexes the predictor's
// checkpoint ring and `gen` makes a handle that outlived its slot detectable.
struct VanadisBranchCheckpoint {
    uint16_t slot = 0xFFFF;
    uint16_t gen  = 0;

    bool valid() const { return slot != 0xFFFF; }
    void clear()
    {
        slot = 0xFFFF;
        gen  = 0;
    }
};

// The return address stack. Pushed at calls and popped at returns, both at
// prediction time, so both are speculative and both are undone by repair.
template <size_t DEPTH>
class VanadisReturnAddressStack
{
public:
    VanadisReturnAddressStack() { reset(); }

    void reset()
    {
        for ( size_t i = 0; i < DEPTH; ++i ) {
            entry[i] = 0;
        }
        top = 0;
    }

    void push(const uint64_t addr)
    {
        entry[top] = addr;
        top        = (top + 1) % DEPTH;
    }

    uint64_t pop()
    {
        top = (top + DEPTH - 1) % DEPTH;
        return entry[top];
    }

    uint64_t entry[DEPTH];
    uint32_t top;
};

// The ring of checkpoints. One record per branch that has been predicted and
// has not yet retired; allocation is in fetch order, release is in retire
// order, and a squash releases the record of the mispredicting branch and
// every record younger than it.
//
// The ring is sized from the reorder buffer, so it can never fill: a branch
// only holds a record while it holds a reorder-buffer entry.
template <typename RECORD>
class VanadisCheckpointRing
{
public:
    VanadisCheckpointRing() : head_(0), tail_(0), count_(0), undecoded_(0), exhausted_(0), stale_(0) {}

    void resize(const uint32_t n)
    {
        entry_.assign(n == 0 ? 1 : n, RECORD());
        gen_.assign(entry_.size(), 1);
        head_ = tail_ = count_ = 0;
        undecoded_ = 0;
    }

    uint32_t capacity() const { return static_cast<uint32_t>(entry_.size()); }
    uint32_t inFlight() const { return count_; }
    uint64_t exhaustedCount() const { return exhausted_; }
    uint64_t staleCount() const { return stale_; }

    // Take the next record. Returns nullptr only if the ring is full, which
    // the reorder buffer makes impossible; the caller treats it as fatal.
    RECORD* allocate(VanadisBranchCheckpoint* handle)
    {
        if ( count_ >= entry_.size() ) {
            exhausted_++;
            handle->clear();
            return nullptr;
        }

        const uint32_t slot = head_;
        head_               = (head_ + 1) % static_cast<uint32_t>(entry_.size());
        count_++;

        handle->slot = static_cast<uint16_t>(slot);
        handle->gen  = gen_[slot];
        undecoded_++;

        return &entry_[slot];
    }

    // THE DECODE STAGE HAS CAUGHT UP WITH THIS RECORD. Records are allocated
    // by the fetch stage, in fetch order, and consumed by the decode stage in
    // the same order; the ones between the two stages are the fetch stage's
    // run-ahead, and they are what a re-steer at decode throws away.
    void markDecoded(const VanadisBranchCheckpoint& handle)
    {
        if ( handleIsLive(handle) && (undecoded_ > 0) ) { undecoded_--; }
    }

    uint32_t undecodedCount() const { return undecoded_; }

    // Throw away the fetch stage's run-ahead: every record allocated after the
    // last one the decode stage consumed. Nothing older is touched, so the
    // branches already in the reorder buffer keep their records.
    uint32_t discardUndecoded()
    {
        const uint32_t n       = static_cast<uint32_t>(entry_.size());
        const uint32_t discard = (undecoded_ > count_) ? count_ : undecoded_;

        for ( uint32_t i = 0; i < discard; ++i ) {
            const uint32_t slot = (head_ + n - 1 - i) % n;
            gen_[slot]          = static_cast<uint16_t>(gen_[slot] + 1);
        }

        head_      = (head_ + n - discard) % n;
        count_    -= discard;
        undecoded_ = 0;
        return discard;
    }

    // Everything younger than this record, decoded or not. The record itself
    // stays live, because the branch that owns it is still in the reorder
    // buffer and has still to retire.
    uint32_t discardYoungerThan(const VanadisBranchCheckpoint& handle)
    {
        if ( !handleIsLive(handle) ) { return 0; }

        const uint32_t n     = static_cast<uint32_t>(entry_.size());
        const uint32_t after = (static_cast<uint32_t>(handle.slot) + 1) % n;

        uint32_t discard = 0;
        while ( head_ != after ) {
            head_ = (head_ + n - 1) % n;
            gen_[head_] = static_cast<uint16_t>(gen_[head_] + 1);
            ++discard;
            if ( discard > n ) { break; }
        }

        count_    -= (discard > count_) ? count_ : discard;
        undecoded_ = 0;
        return discard;
    }

    bool handleIsLive(const VanadisBranchCheckpoint& handle) const
    {
        if ( !handle.valid() ) { return false; }
        if ( handle.slot >= entry_.size() ) { return false; }
        return gen_[handle.slot] == handle.gen;
    }

    // The record a handle names. Valid until the slot is handed out again,
    // which is why retire() may leave the contents alone.
    RECORD& record(const VanadisBranchCheckpoint& handle) { return entry_[handle.slot]; }

    // The oldest record leaves the ring. Its contents stay readable until the
    // slot is reallocated, so a repair that follows in the same cycle can
    // still use them.
    bool retire(const VanadisBranchCheckpoint& handle)
    {
        if ( !handleIsLive(handle) || (count_ == 0) || (handle.slot != tail_) ) {
            stale_++;
            return false;
        }

        gen_[tail_] = static_cast<uint16_t>(gen_[tail_] + 1);
        tail_       = (tail_ + 1) % static_cast<uint32_t>(entry_.size());
        count_--;
        return true;
    }

    // Records still in the ring, newest first. After retire() of the
    // mispredicting branch these are exactly the wrong-path branches.
    uint32_t youngerCount() const { return count_; }
    uint32_t tailSlot() const { return tail_; }
    RECORD&  slotRecord(const uint32_t slot) { return entry_[slot]; }
    uint32_t headSlot() const { return head_; }

    // Records allocated after this one and still live.
    uint32_t countYoungerThan(const VanadisBranchCheckpoint& handle) const
    {
        if ( !handleIsLive(handle) ) { return 0; }
        const uint32_t n     = static_cast<uint32_t>(entry_.size());
        const uint32_t after = (static_cast<uint32_t>(handle.slot) + 1) % n;
        return (head_ + n - after) % n;
    }

    RECORD& younger(const uint32_t i)
    {
        // i == 0 is the youngest.
        const uint32_t n = static_cast<uint32_t>(entry_.size());
        return entry_[(head_ + n - 1 - i) % n];
    }

    // Everything unretired is gone.
    void discardAll()
    {
        const uint32_t n = static_cast<uint32_t>(entry_.size());
        for ( uint32_t i = 0; i < count_; ++i ) {
            const uint32_t slot = (tail_ + i) % n;
            gen_[slot]          = static_cast<uint16_t>(gen_[slot] + 1);
        }
        head_      = tail_;
        count_     = 0;
        undecoded_ = 0;
    }

private:
    std::vector<RECORD>   entry_;
    std::vector<uint16_t> gen_;
    uint32_t              head_;
    uint32_t              tail_;
    uint32_t              count_;
    uint32_t              undecoded_;
    uint64_t              exhausted_;
    uint64_t              stale_;
};

} // namespace Vanadis
} // namespace SST

#endif
