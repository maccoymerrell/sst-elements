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

#ifndef _H_VANADIS_INSTRUCTION
#define _H_VANADIS_INSTRUCTION

#include "decoder/visaopts.h"
#include "inst/regfile.h"
#include "inst/regstack.h"
#include "inst/vinsttype.h"
#include "inst/vregfmt.h"

#include <cstring>
#include <map>
#include <vector>
#include <sst/core/output.h>
#include <string>
#include <sstream>



namespace SST {
namespace Vanadis {

// How many architectural registers of one kind the issue stage's register
// scoreboard can hold in a machine word. RISC-V declares 35 integer and 32
// floating-point; MIPS declares 34 of each. VanadisComponent checks its
// decoders against this at construction and refuses to start rather than
// silently dropping a register out of the dependency check.
static constexpr uint16_t VANADIS_MASKED_ISA_REGS = 64;

// WHAT AN INSTRUCTION IS, ASKED WITHOUT dynamic_cast.
//
// Every one of these classes derives from VanadisInstruction VIRTUALLY, which
// is why the pipeline could only ask "is this a load?" with dynamic_cast:
// static_cast cannot go down from a virtual base at all. libstdc++ answers a
// dynamic_cast by walking the type graph -- __do_dyncast -- and the issue,
// dispatch, retire and load/store-queue paths ask on every memory instruction,
// every branch and every fence, which put __dynamic_cast at 3.3 % of the whole
// simulator. The `as*()` hooks below answer the same question with one virtual
// call and a compiler-computed this-adjustment. Each returns exactly what the
// corresponding dynamic_cast returned, including nullptr.
class VanadisLoadInstruction;
class VanadisStoreInstruction;
class VanadisStoreConditionalInstruction;
class VanadisFenceInstruction;
class VanadisSpeculatedInstruction;
class VanadisSysCallInstruction;

// WHERE A DYNAMIC INSTRUCTION'S MEMORY COMES FROM.
//
// The decoder clones a cached bundle entry for every instruction it decodes and
// the core deletes it when it retires or is squashed, so the simulator asks the
// C library for one object per dynamic instruction and gives it back a few
// thousand cycles later -- 9.5 million round trips in eight simulated
// milliseconds on the hash-table point, and that is before the eight register
// lists each one used to allocate separately. malloc and free together were
// 11.7 % of the whole simulator.
//
// These blocks are all one of a handful of sizes and their lifetime is bounded
// by the reorder buffer, so a free list per size class is the whole story: an
// allocation is a pop, a free is a push, and nothing is returned to the C
// library. A sixteen-byte header carries the size class, which both keeps the
// natural alignment of ::operator new and gives the free list its link word.
//
// The lists are thread-local, so a partitioned run has one per thread. A block
// freed on a thread other than the one that allocated it is still the right
// size for that thread's class of the same index, so the pools stay correct if
// they drift in size.
class VanadisInstructionArena
{
    public:
        static constexpr size_t kGranule   = 16;   // preserves max_align_t alignment
        static constexpr size_t kMaxBucket = 64;   // objects up to 1008 bytes are pooled

        static void* alloc(size_t sz)
        {
            const size_t b = (sz + kGranule - 1) / kGranule + 1;  // granules, header included

            if ( b > kMaxBucket ) {
                char* raw = static_cast<char*>(::operator new(b * kGranule));
                reinterpret_cast<size_t*>(raw)[0] = 0;            // 0: came from the C library
                return raw + kGranule;
            }

            void** heads = freeHeads();
            char*  raw;

            if ( heads[b] != nullptr ) {
                raw      = static_cast<char*>(heads[b]);
                heads[b] = reinterpret_cast<void**>(raw)[1];
            }
            else {
                raw = static_cast<char*>(::operator new(b * kGranule));
            }

            reinterpret_cast<size_t*>(raw)[0] = b;
            return raw + kGranule;
        }

        static void release(void* p)
        {
            if ( nullptr == p ) { return; }

            char*        raw = static_cast<char*>(p) - kGranule;
            const size_t b   = reinterpret_cast<size_t*>(raw)[0];

            if ( 0 == b ) { ::operator delete(raw); return; }

            void** heads = freeHeads();
            reinterpret_cast<void**>(raw)[1] = heads[b];
            heads[b]                         = raw;
        }

    private:
        static void** freeHeads()
        {
            static thread_local void* heads[kMaxBucket + 1] = {};
            return heads;
        }
};

// How many register-list entries an instruction keeps inside itself. The eight
// lists together are six to eight entries for a normal RISC-V instruction, so
// this covers every one of them and the heap path below is for an ISA that
// names more. It replaces eight separate new uint16_t[] per instruction.
static constexpr uint16_t VANADIS_INLINE_REG_SLOTS = 16;

class VanadisInstruction
{
    public:
        VanadisInstruction(const uint64_t address, const uint32_t hw_thr, const VanadisDecoderOptions* isa_opts,
            const uint16_t c_phys_int_reg_in, const uint16_t c_phys_int_reg_out, const uint16_t c_isa_int_reg_in,
            const uint16_t c_isa_int_reg_out, const uint16_t c_phys_fp_reg_in, const uint16_t c_phys_fp_reg_out,
            const uint16_t c_isa_fp_reg_in, const uint16_t c_isa_fp_reg_out) :
            ins_address(address),
            hw_thread(hw_thr),
            isa_options(isa_opts),
            count_phys_int_reg_in(c_phys_int_reg_in),
            count_phys_int_reg_out(c_phys_int_reg_out),
            count_isa_int_reg_in(c_isa_int_reg_in),
            count_isa_int_reg_out(c_isa_int_reg_out),
            count_phys_fp_reg_in(c_phys_fp_reg_in),
            count_phys_fp_reg_out(c_phys_fp_reg_out),
            count_isa_fp_reg_in(c_isa_fp_reg_in),
            count_isa_fp_reg_out(c_isa_fp_reg_out)
        {
            // One block for all eight register lists, inside the instruction
            // when it fits. This used to be eight new uint16_t[] and eight
            // memsets per instruction, and the counts are single digits.
            bindRegisterLists(true);

            trap_error_           = false;
            has_executed_         = false;
            has_issued_           = false;
            has_renamed_          = false;
            end_uop_group_        = false;
            is_front_of_rob_      = false;
            has_rob_slot_         = false;
            isa_reg_masks_valid_  = false;
            sw_thread             = hw_thr;
        }

        virtual ~VanadisInstruction()
        {
            if ( reg_heap_ != nullptr ) { delete[] reg_heap_; }
        }

        VanadisInstruction(const VanadisInstruction& copy_me) :
            ins_address(copy_me.ins_address),
            hw_thread(copy_me.hw_thread),
            isa_options(copy_me.isa_options),
            count_phys_int_reg_in(copy_me.count_phys_int_reg_in),
            count_phys_int_reg_out(copy_me.count_phys_int_reg_out),
            count_isa_int_reg_in(copy_me.count_isa_int_reg_in),
            count_isa_int_reg_out(copy_me.count_isa_int_reg_out),
            count_phys_fp_reg_in(copy_me.count_phys_fp_reg_in),
            count_phys_fp_reg_out(copy_me.count_phys_fp_reg_out),
            count_isa_fp_reg_in(copy_me.count_isa_fp_reg_in),
            count_isa_fp_reg_out(copy_me.count_isa_fp_reg_out)
        {
            trap_error_           = copy_me.trap_error_;
            has_executed_         = copy_me.has_executed_;
            has_issued_           = copy_me.has_issued_;
            has_renamed_          = copy_me.has_renamed_;
            end_uop_group_        = copy_me.end_uop_group_;
            is_front_of_rob_      = false;
            has_rob_slot_         = false;
            // Not copied: the clone's register lists are rebuilt below, and a
            // derived copy constructor may rewrite them again afterwards.
            isa_reg_masks_valid_  = false;
            sw_thread             = copy_me.sw_thread;

            // One block, then one memcpy: the source's eight lists are
            // contiguous in the same order, so the clone is a straight copy.
            bindRegisterLists(false);
            std::memcpy(regBlock(), copy_me.regBlock(), totalRegSlots() * sizeof( uint16_t ));
        }

        // different
        void writeIntRegs(char* buffer, size_t max_buff_size)
        {
            size_t index_so_far = 0;

            index_so_far += snprintf(&buffer[index_so_far], max_buff_size - index_so_far, "in: { ");

            if ( count_isa_int_reg_in > 0 ) {
                index_so_far +=
                    snprintf(&buffer[index_so_far], max_buff_size - index_so_far, "%" PRIu16 "", isa_int_regs_in[0]);

                for ( int i = 1; i < count_isa_int_reg_in; ++i ) {
                    index_so_far +=
                        snprintf(&buffer[index_so_far], max_buff_size - index_so_far, ", %" PRIu16 "", isa_int_regs_in[i]);
                }
            }

            index_so_far += snprintf(&buffer[index_so_far], max_buff_size - index_so_far, " } -> { ");

            if ( count_phys_int_reg_in > 0 ) {
                index_so_far +=
                    snprintf(&buffer[index_so_far], max_buff_size - index_so_far, "%" PRIu16 "", phys_int_regs_in[0]);

                for ( int i = 1; i < count_phys_int_reg_in; ++i ) {
                    index_so_far +=
                        snprintf(&buffer[index_so_far], max_buff_size - index_so_far, ", %" PRIu16 "", phys_int_regs_in[i]);
                }
            }

            index_so_far += snprintf(&buffer[index_so_far], max_buff_size - index_so_far, " } / out: { ");

            if ( count_isa_int_reg_out > 0 ) {
                index_so_far +=
                    snprintf(&buffer[index_so_far], max_buff_size - index_so_far, "%" PRIu16 "", isa_int_regs_out[0]);

                for ( int i = 1; i < count_isa_int_reg_out; ++i ) {
                    index_so_far +=
                        snprintf(&buffer[index_so_far], max_buff_size - index_so_far, ", %" PRIu16 "", isa_int_regs_out[i]);
                }
            }

            index_so_far += snprintf(&buffer[index_so_far], max_buff_size - index_so_far, " } -> { ");

            if ( count_phys_int_reg_out > 0 ) {
                index_so_far +=
                    snprintf(&buffer[index_so_far], max_buff_size - index_so_far, "%" PRIu16 "", phys_int_regs_out[0]);

                for ( int i = 1; i < count_phys_int_reg_out; ++i ) {
                    index_so_far += snprintf(
                        &buffer[index_so_far], max_buff_size - index_so_far, ", %" PRIu16 "", phys_int_regs_out[i]);
                }
            }

            index_so_far += snprintf(&buffer[index_so_far], max_buff_size - index_so_far, " }");
        }

        // different
        void writeFPRegs(char* buffer, size_t max_buff_size)
        {
            size_t index_so_far = 0;

            index_so_far += snprintf(&buffer[index_so_far], max_buff_size - index_so_far, "in: { ");

            if ( count_isa_fp_reg_in > 0 ) {
                index_so_far +=
                    snprintf(&buffer[index_so_far], max_buff_size - index_so_far, "%" PRIu16 "", isa_fp_regs_in[0]);

                for ( int i = 1; i < count_isa_fp_reg_in; ++i ) {
                    index_so_far +=
                        snprintf(&buffer[index_so_far], max_buff_size - index_so_far, ", %" PRIu16 "", isa_fp_regs_in[i]);
                }
            }

            index_so_far += snprintf(&buffer[index_so_far], max_buff_size - index_so_far, " } -> { ");

            if ( count_phys_fp_reg_in > 0 ) {
                index_so_far +=
                    snprintf(&buffer[index_so_far], max_buff_size - index_so_far, "%" PRIu16 "", phys_fp_regs_in[0]);

                for ( int i = 1; i < count_phys_fp_reg_in; ++i ) {
                    index_so_far +=
                        snprintf(&buffer[index_so_far], max_buff_size - index_so_far, ", %" PRIu16 "", phys_fp_regs_in[i]);
                }
            }

            index_so_far += snprintf(&buffer[index_so_far], max_buff_size - index_so_far, " } / out: { ");

            if ( count_isa_fp_reg_out > 0 ) {
                index_so_far +=
                    snprintf(&buffer[index_so_far], max_buff_size - index_so_far, "%" PRIu16 "", isa_fp_regs_out[0]);

                for ( int i = 1; i < count_isa_fp_reg_out; ++i ) {
                    index_so_far +=
                        snprintf(&buffer[index_so_far], max_buff_size - index_so_far, ", %" PRIu16 "", isa_fp_regs_out[i]);
                }
            }

            index_so_far += snprintf(&buffer[index_so_far], max_buff_size - index_so_far, " } -> { ");

            if ( count_phys_fp_reg_out > 0 ) {
                index_so_far +=
                    snprintf(&buffer[index_so_far], max_buff_size - index_so_far, "%" PRIu16 "", phys_fp_regs_out[0]);

                for ( int i = 1; i < count_phys_fp_reg_out; ++i ) {
                    index_so_far +=
                        snprintf(&buffer[index_so_far], max_buff_size - index_so_far, ", %" PRIu16 "", phys_fp_regs_out[i]);
                }
            }

            index_so_far += snprintf(&buffer[index_so_far], max_buff_size - index_so_far, " }");
        }


        uint16_t countPhysIntRegIn() const { return count_phys_int_reg_in; }
        uint16_t countPhysIntRegOut() const { return count_phys_int_reg_out; }
        uint16_t countPhysFPRegIn() const { return count_phys_fp_reg_in; }
        uint16_t countPhysFPRegOut() const { return count_phys_fp_reg_out; }


        uint16_t countISAIntRegIn() const { return count_isa_int_reg_in; }
        uint16_t countISAIntRegOut() const { return count_isa_int_reg_out; }
        uint16_t countISAFPRegIn() const { return count_isa_fp_reg_in; }
        uint16_t countISAFPRegOut() const { return count_isa_fp_reg_out; }

        uint16_t getPhysIntRegIn(const uint16_t index) const { return phys_int_regs_in[index]; }
        uint16_t getPhysIntRegOut(const uint16_t index) const { return phys_int_regs_out[index]; }
        uint16_t getISAIntRegIn(const uint16_t index) const { return isa_int_regs_in[index]; }
        uint16_t getISAIntRegOut(const uint16_t index) const { return isa_int_regs_out[index]; }


        uint16_t getPhysFPRegIn(const uint16_t index) const { return phys_fp_regs_in[index]; }
        uint16_t getPhysFPRegOut(const uint16_t index) const { return phys_fp_regs_out[index]; }
        uint16_t getISAFPRegIn(const uint16_t index) const { return isa_fp_regs_in[index]; }
        uint16_t getISAFPRegOut(const uint16_t index) const { return isa_fp_regs_out[index]; }

        // The same four register lists as bit sets. See the note on the mask
        // members: these are what the issue stage's scoreboard checks read, so
        // that checking an instruction against every older one in the reorder
        // buffer is four ANDs rather than four loops over heap arrays.
        uint64_t getISAIntRegInMask()  { ensureISARegMasks(); return isa_int_reg_in_mask_; }
        uint64_t getISAIntRegOutMask() { ensureISARegMasks(); return isa_int_reg_out_mask_; }
        uint64_t getISAFPRegInMask()   { ensureISARegMasks(); return isa_fp_reg_in_mask_; }
        uint64_t getISAFPRegOutMask()  { ensureISARegMasks(); return isa_fp_reg_out_mask_; }

        void setPhysIntRegIn(const uint16_t index, const uint16_t reg) { phys_int_regs_in[index] = reg; }
        void setPhysIntRegOut(const uint16_t index, const uint16_t reg) { phys_int_regs_out[index] = reg; }
        void setPhysFPRegIn(const uint16_t index, const uint16_t reg) { phys_fp_regs_in[index] = reg; }
        void setPhysFPRegOut(const uint16_t index, const uint16_t reg) { phys_fp_regs_out[index] = reg; }

        // See the note on VanadisInstructionArena above. Every derived class
        // inherits these, so `new VanadisXxxInstruction(...)` and the virtual
        // destructor's `delete` both go to the pool.
        static void* operator new(size_t sz) { return VanadisInstructionArena::alloc(sz); }
        static void  operator delete(void* p) noexcept { VanadisInstructionArena::release(p); }

        virtual VanadisInstruction* clone() = 0;

        // See the note above the forward declarations at the top of this file.
        virtual VanadisLoadInstruction*             asLoad()             { return nullptr; }
        virtual VanadisStoreInstruction*            asStore()            { return nullptr; }
        virtual VanadisStoreConditionalInstruction* asStoreConditional() { return nullptr; }
        virtual VanadisFenceInstruction*            asFence()            { return nullptr; }
        virtual VanadisSpeculatedInstruction*       asSpeculated()       { return nullptr; }
        virtual VanadisSysCallInstruction*          asSysCall()          { return nullptr; }

        void markEndOfMicroOpGroup() { end_uop_group_ = true; }
        bool endsMicroOpGroup() const { return end_uop_group_; }
        bool trapsError() const { return trap_error_; }

        uint64_t getInstructionAddress() const { return ins_address; }
        uint32_t getHWThread() const { return hw_thread; }

        void setSWThread(uint32_t thr) { sw_thread=thr;}
        uint32_t getSWThread() {return sw_thread;}

        virtual const char* getInstCode() const = 0 ;

        virtual void printToBuffer(char* buffer, size_t buffer_size) { snprintf(buffer, buffer_size, "%s", getInstCode()); }

        virtual VanadisFunctionalUnitType getInstFuncType() const = 0;


        virtual void instOp(VanadisRegisterFile* regFile,
                            uint16_t phys_int_regs_out_0, uint16_t phys_int_regs_in_0,
                            uint16_t phys_int_regs_in_1)
        {
            ;
        }

        virtual void instOp(VanadisRegisterFile* regFile,
                                uint16_t phys_int_regs_out_0, uint16_t phys_int_regs_in_0)
        {
            ;
        }

        virtual void scalarExecute(SST::Output* output, VanadisRegisterFile* regFile)
        {
            uint16_t phys_int_regs_out_0 = getPhysIntRegOut(0);
            uint16_t phys_int_regs_in_0 = getPhysIntRegIn(0);
            uint16_t phys_int_regs_in_1 = getPhysIntRegIn(1);
            log(output, 16, 65535,phys_int_regs_out_0,phys_int_regs_in_0,phys_int_regs_in_1);
            instOp(regFile,phys_int_regs_out_0, phys_int_regs_in_0, phys_int_regs_in_1);
            markExecuted();
        }

        virtual void log(SST::Output* output, int verboselevel, uint16_t sw_thr,
                uint16_t phys_int_regs_out_0,uint16_t phys_int_regs_in_0)
        {
            #ifdef VANADIS_BUILD_DEBUG
            if(output->getVerboseLevel() >= verboselevel) {

                std::ostringstream ss;
                ss << "hw_thr="<<getHWThread()<<" sw_thr=" <<sw_thr;
                ss << " Execute: 0x" << std::hex << getInstructionAddress() << std::dec << " " << getInstCode();
                ss << " phys: out=" <<  phys_int_regs_out_0 << " in=" << phys_int_regs_in_0;
                // ss << " imm=" << imm_value;
                ss << ", isa: out=" <<  isa_int_regs_out[0]  << " in=" << isa_int_regs_in[0];
                output->verbose( CALL_INFO, verboselevel, 0, "%s\n", ss.str().c_str());
            }
            #endif
        }

        virtual void  execute(SST::Output* output, std::vector<VanadisRegisterFile*>& regFiles)
        {
            scalarExecute(output, regFiles[hw_thread]);
        }

        virtual void log(SST::Output* output, int verboselevel, uint16_t sw_thr,
                            uint16_t phys_int_regs_out_0,uint16_t phys_int_regs_in_0,
                                    uint16_t phys_int_regs_in_1)
        {
            #ifdef VANADIS_BUILD_DEBUG
            if(output->getVerboseLevel() >= verboselevel) {
                std::string instcode = getInstCode();
                std::string fpinst = "FP";
                if(instcode.find(fpinst) != std::string::npos )
                {
                    output->verbose(
                    CALL_INFO, verboselevel, 0,
                    "hw_thr=%d sw_thr = %d Execute: 0x%" PRI_ADDR " %s phys: out=%" PRIu16 " in=%" PRIu16 ", %" PRIu16 ", isa: out=%" PRIu16
                    " / in=%" PRIu16 ", %" PRIu16 "\n",
                    getHWThread(),sw_thr, getInstructionAddress(), getInstCode(), phys_int_regs_out_0, phys_int_regs_in_0,
                    phys_int_regs_in_1, isa_fp_regs_out[0], isa_fp_regs_in[0], isa_fp_regs_in[1]);
                }
                else
                output->verbose(
                    CALL_INFO, verboselevel, 0,
                    "hw_thr=%d sw_thr = %d Execute: 0x%" PRI_ADDR " %s phys: out=%" PRIu16 " in=%" PRIu16 ", %" PRIu16 ", isa: out=%" PRIu16
                    " / in=%" PRIu16 ", %" PRIu16 "\n",
                    getHWThread(),sw_thr, getInstructionAddress(), getInstCode(), phys_int_regs_out_0, phys_int_regs_in_0,
                    phys_int_regs_in_1, isa_int_regs_out[0], isa_int_regs_in[0], isa_int_regs_in[1]);
            }
            #endif
        }


        virtual void print(SST::Output* output) { output->verbose(CALL_INFO, 8, 0, "%s", getInstCode()); }

        // Is the instruction predicted (speculation point).
        // for normal instructions this is false
        // but branches and jumps will get predicte
        virtual bool isSpeculated() const { return false; }

        bool completedExecution() const { return has_executed_; }
        bool completedIssue() const { return has_issued_; }

        // RENAMED IS NOT ISSUED, NOW THAT THE TWO ARE SEPARATE STAGES.
        //
        // completedIssue() still means what it has always meant: this
        // instruction has been given a functional unit, an LSQ slot or a
        // coprocessor queue entry, and every caller that asks the question is
        // asking that. completedRename() is the new, earlier fact: the
        // instruction has been through dispatch, so it owns physical registers
        // and mis-speculation has to hand them back. Between the two it is
        // sitting in the scheduling window waiting for its operands.
        bool completedRename() const { return has_renamed_; }
        void markRenamed() { has_renamed_ = true; }

        // WAKING THE CONSUMERS IS DONE HERE, AND NOWHERE ELSE.
        //
        // An instruction becomes readable by its dependents at the instant its
        // result is written back, and there are a dozen places that happens: a
        // functional unit finishing, a load response, a store completing, a
        // fence draining, a coprocessor answering, a syscall returning, and the
        // zero-latency classes that complete at issue. Rather than trust twelve
        // call sites to stay complete as instruction classes are added, the
        // single definition of "this instruction has produced its result" puts
        // itself on the core's write-back queue, which the issue stage drains
        // at the top of the next cycle. markExecuted() was virtual and had no
        // overrides anywhere in the tree, so nothing is lost by making it the
        // only definition there can be.
        //
        // The guard is not defensive: several paths mark an instruction
        // executed more than once, and a second entry would wake its consumers
        // twice and corrupt the waiter lists.
        void markExecuted()
        {
            if ( LIKELY(!has_executed_) ) {
                has_executed_ = true;
                if ( LIKELY(nullptr != writeback_q_) ) { writeback_q_->push_back(this); }
            }
        }

        // Set at dispatch, which is the earliest an instruction can execute.
        void setWritebackQueue(std::vector<VanadisInstruction*>* q) { writeback_q_ = q; }

        void markIssued() { has_issued_ = true; }

        bool checkFrontOfROB() const { return is_front_of_rob_; }
        void markFrontOfROB() { is_front_of_rob_ = true; }

        bool has_rob_slot_Issued() const { return has_rob_slot_; }
        void markROBSlotIssued() { has_rob_slot_ = true; }

        const VanadisDecoderOptions* getISAOptions() const { return isa_options; }

        void flagError() { trap_error_ = true; }

        virtual bool performIntRegisterRecovery() const { return true; }
        virtual bool performFPRegisterRecovery() const { return true; }

        virtual bool updatesFPFlags() const { return false; }
        virtual void updateFPFlags() {}

        virtual void returnOutRegs( VanadisRegisterStack* int_stack, VanadisRegisterStack* fp_stack )
        {
            for ( auto i = 0; i < countPhysIntRegOut(); i++ ) {
                // The hardwired-zero architectural register is not renamed and
                // its physical register is not the free list's to hand out; an
                // instruction that "writes" it took nothing and has nothing to
                // give back.
                if ( UNLIKELY(isa_int_regs_out[i] == isa_options->getRegisterIgnoreWrites()) ) { continue; }
                int_stack->push( getPhysIntRegOut(i) );
            }
            for ( auto i = 0; i < countPhysFPRegOut(); i++ ) {
                fp_stack->push( getPhysFPRegOut(i) );
            }
        }

        uint16_t getNumStores()
        {
            return 0;
        }



    protected:

        // THE EIGHT REGISTER LISTS LIVE IN ONE BLOCK, in this order:
        //   isa_int_in, isa_int_out, isa_fp_in, isa_fp_out,
        //   phys_int_in, phys_int_out, phys_fp_in, phys_fp_out
        // inside the instruction when they fit (they always do on RISC-V and
        // MIPS) and on the heap when they do not. A list whose count is zero
        // keeps its null pointer, as it did when each was allocated on its own.
        uint16_t totalRegSlots() const
        {
            return static_cast<uint16_t>(
                count_isa_int_reg_in + count_isa_int_reg_out + count_isa_fp_reg_in + count_isa_fp_reg_out +
                count_phys_int_reg_in + count_phys_int_reg_out + count_phys_fp_reg_in + count_phys_fp_reg_out);
        }

        uint16_t*       regBlock()       { return (reg_heap_ != nullptr) ? reg_heap_ : reg_inline_; }
        const uint16_t* regBlock() const { return (reg_heap_ != nullptr) ? reg_heap_ : reg_inline_; }

        void bindRegisterLists(const bool zero_fill)
        {
            const uint16_t total = totalRegSlots();

            reg_heap_ = (total > VANADIS_INLINE_REG_SLOTS) ? new uint16_t[total] : nullptr;

            uint16_t* p = regBlock();
            if ( zero_fill ) { std::memset(p, 0, total * sizeof( uint16_t )); }

            isa_int_regs_in   = (count_isa_int_reg_in   > 0) ? p : nullptr; p += count_isa_int_reg_in;
            isa_int_regs_out  = (count_isa_int_reg_out  > 0) ? p : nullptr; p += count_isa_int_reg_out;
            isa_fp_regs_in    = (count_isa_fp_reg_in    > 0) ? p : nullptr; p += count_isa_fp_reg_in;
            isa_fp_regs_out   = (count_isa_fp_reg_out   > 0) ? p : nullptr; p += count_isa_fp_reg_out;
            phys_int_regs_in  = (count_phys_int_reg_in  > 0) ? p : nullptr; p += count_phys_int_reg_in;
            phys_int_regs_out = (count_phys_int_reg_out > 0) ? p : nullptr; p += count_phys_int_reg_out;
            phys_fp_regs_in   = (count_phys_fp_reg_in   > 0) ? p : nullptr; p += count_phys_fp_reg_in;
            phys_fp_regs_out  = (count_phys_fp_reg_out  > 0) ? p : nullptr;
        }

        // Change the INTEGER list sizes after the base constructor has run.
        // VanadisPartialLoadInstruction is the only caller: it needs a second
        // input register that the load it derives from does not have. The
        // floating-point lists keep their contents; the integer lists come back
        // zeroed, which is what the base constructor left them as and what the
        // code that used to delete and re-new them relied on.
        void reshapeIntRegisterLists(
            const uint16_t isa_in, const uint16_t isa_out, const uint16_t phys_in, const uint16_t phys_out)
        {
            std::vector<uint16_t> keep_isa_fp_in(isa_fp_regs_in, isa_fp_regs_in + count_isa_fp_reg_in);
            std::vector<uint16_t> keep_isa_fp_out(isa_fp_regs_out, isa_fp_regs_out + count_isa_fp_reg_out);
            std::vector<uint16_t> keep_phys_fp_in(phys_fp_regs_in, phys_fp_regs_in + count_phys_fp_reg_in);
            std::vector<uint16_t> keep_phys_fp_out(phys_fp_regs_out, phys_fp_regs_out + count_phys_fp_reg_out);

            if ( reg_heap_ != nullptr ) { delete[] reg_heap_; reg_heap_ = nullptr; }

            count_isa_int_reg_in   = isa_in;
            count_isa_int_reg_out  = isa_out;
            count_phys_int_reg_in  = phys_in;
            count_phys_int_reg_out = phys_out;

            bindRegisterLists(true);

            for ( uint16_t i = 0; i < count_isa_fp_reg_in; ++i )   { isa_fp_regs_in[i]   = keep_isa_fp_in[i]; }
            for ( uint16_t i = 0; i < count_isa_fp_reg_out; ++i )  { isa_fp_regs_out[i]  = keep_isa_fp_out[i]; }
            for ( uint16_t i = 0; i < count_phys_fp_reg_in; ++i )  { phys_fp_regs_in[i]  = keep_phys_fp_in[i]; }
            for ( uint16_t i = 0; i < count_phys_fp_reg_out; ++i ) { phys_fp_regs_out[i] = keep_phys_fp_out[i]; }

            isa_reg_masks_valid_ = false;
        }

        void ensureISARegMasks()
        {
            if ( !isa_reg_masks_valid_ ) { buildISARegMasks(); }
        }

        void buildISARegMasks()
        {
            isa_int_reg_in_mask_  = isaRegSetMask(isa_int_regs_in, count_isa_int_reg_in);
            isa_int_reg_out_mask_ = isaRegSetMask(isa_int_regs_out, count_isa_int_reg_out);
            isa_fp_reg_in_mask_   = isaRegSetMask(isa_fp_regs_in, count_isa_fp_reg_in);
            isa_fp_reg_out_mask_  = isaRegSetMask(isa_fp_regs_out, count_isa_fp_reg_out);
            isa_reg_masks_valid_  = true;
        }

        static uint64_t isaRegSetMask(const uint16_t* regs, const uint16_t count)
        {
            uint64_t mask = 0;
            for ( uint16_t i = 0; i < count; ++i ) {
                const uint16_t reg = regs[i];
                // A register number this large cannot be in the mask at all.
                // The component refuses to start on an ISA that declares that
                // many registers, so reaching here means a corrupt register
                // list, which assignRegistersToInstruction fatals on when the
                // instruction gets to issue.
                if ( reg < VANADIS_MASKED_ISA_REGS ) { mask |= (static_cast<uint64_t>(1) << reg); }
            }
            return mask;
        }

        // MEMBER ORDER HERE IS DELIBERATE, AND IT IS ABOUT THE ISSUE STAGE.
        //
        // The issue stage walks EVERY entry of the reorder buffer every cycle,
        // and for most of them it only wants to know four things: has this one
        // issued, and which architectural registers does it read and write.
        // Everything needed to answer that -- the vtable pointer, the four
        // masks, the two output counts, has_issued_ -- is kept inside the first
        // 64 bytes of the object, so walking past a ROB entry touches ONE cache
        // line. It used to touch the object plus four separately-allocated
        // uint16_t arrays. At 352 entries a cycle that is the whole cost of the
        // stage.
        const uint64_t ins_address;

        // The ISA registers this instruction reads and writes, as bit sets --
        // one bit per architectural register, which is why the component
        // refuses to start on an ISA with more than VANADIS_MASKED_ISA_REGS of
        // them (vanadis.cc). The scoreboard checks at issue are then one AND
        // each instead of a loop over a heap array.
        //
        // BUILT LAZILY, and that is not an optimisation: a DERIVED constructor
        // may still be rewriting the register lists after this one has run --
        // VanadisPartialLoadInstruction deletes and reallocates both of its
        // integer lists and changes their counts -- so there is no point during
        // construction at which the base class may read them. Nothing asks for
        // a mask before the instruction reaches issue, which is long after any
        // constructor has finished.
        uint64_t isa_int_reg_in_mask_;
        uint64_t isa_int_reg_out_mask_;
        uint64_t isa_fp_reg_in_mask_;
        uint64_t isa_fp_reg_out_mask_;

        uint16_t count_isa_int_reg_in;
        uint16_t count_isa_int_reg_out;
        uint16_t count_isa_fp_reg_in;
        uint16_t count_isa_fp_reg_out;

        bool isa_reg_masks_valid_;
        bool trap_error_ = false;
        bool has_executed_;
        bool has_issued_;
        bool end_uop_group_;
        bool is_front_of_rob_;
        bool has_rob_slot_;
        bool has_renamed_;

        const uint32_t hw_thread;

        uint16_t count_phys_int_reg_in;
        uint16_t count_phys_int_reg_out;
        uint16_t count_phys_fp_reg_in;
        uint16_t count_phys_fp_reg_out;

        const VanadisDecoderOptions* isa_options;
        uint32_t sw_thread;

        // The core's write-back queue, handed to the instruction at dispatch.
        // Deliberately NOT in the first cache line: markExecuted() runs a
        // couple of times a cycle, where the first line is what the stages that
        // walk instructions read.
        std::vector<VanadisInstruction*>* writeback_q_ = nullptr;

        uint16_t* isa_int_regs_in;
        uint16_t* isa_int_regs_out;
        uint16_t* isa_fp_regs_in;
        uint16_t* isa_fp_regs_out;

        uint16_t* phys_int_regs_in;
        uint16_t* phys_int_regs_out;
        uint16_t* phys_fp_regs_in;
        uint16_t* phys_fp_regs_out;

        // The storage the eight pointers above point into. Kept last so that
        // nothing above it moves out of the first cache line.
        uint16_t* reg_heap_ = nullptr;                     // null when inline storage is used
        uint16_t  reg_inline_[VANADIS_INLINE_REG_SLOTS];


};

} // namespace Vanadis
} // namespace SST

#endif
