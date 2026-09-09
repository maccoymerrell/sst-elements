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
//
// ---------------------------------------------------------------------------
// A hashed perceptron, in the O-GEHL class: sixteen tables of signed weights,
// each indexed by a hash of the branch address with a different length of
// global history, and the prediction is the sign of the sum of the sixteen
// weights that come back. Training is the perceptron rule -- add the direction
// to every weight that was read -- applied on a misprediction or on a correct
// prediction whose sum was too small to be trusted, with the threshold for
// "too small" adjusted dynamically.
//
// The weights and the threshold are trained only at retirement. What the port
// adds to the reference is that the sixteen shift registers exist twice, as a
// speculative copy advanced when a branch is predicted and an architected copy
// advanced when it retires, and that the sixteen table indices and the sum --
// which the reference keeps in one member because it only ever has one
// prediction outstanding -- live in a per-branch record instead.
// ---------------------------------------------------------------------------

#ifndef _H_VANADIS_BRANCH_UNIT_PERCEPTRON
#define _H_VANADIS_BRANCH_UNIT_PERCEPTRON

#include "vbranch/vbranchcheckpoint.h"

#include <cstdint>
#include <cstring>
#include <vector>

namespace SST {
namespace Vanadis {

class VanadisPerceptronCore
{
public:
    enum : int {
        NTABLES          = 16,
        TABLE_INDEX_BITS = 12,
        TABLE_SIZE       = (1 << TABLE_INDEX_BITS),
        THRESHOLD        = 1,
        SPEED            = 18, // mispredictions between threshold steps
        WEIGHT_MAX       = 127,
        WEIGHT_MIN       = -128,
        WORD_LEN         = TABLE_INDEX_BITS,
        WORDS_PER_VALUE  = 64 / WORD_LEN, // five 12-bit words to a 64-bit value
        VALUE_LEN        = WORD_LEN * WORDS_PER_VALUE,
        MAX_WORDS        = 4,
        RAS_DEPTH        = 32
    };

    // The geometric history lengths. The first table sees no history at all,
    // so it holds the per-address bias.
    static const int* historyLengths()
    {
        static const int len[NTABLES] = {0, 3, 4, 6, 8, 10, 14, 19, 26, 36, 49, 67, 91, 125, 170, 232};
        return len;
    }

    // A shift register that folds its bits into one twelve-bit word before it
    // is used as an index.
    struct FoldedShiftRegister {
        uint64_t words[MAX_WORDS] = {0, 0, 0, 0};
        int      nwords           = 0;
        uint64_t last_mask        = 0;

        void init(const int length)
        {
            const int rem = length % VALUE_LEN;
            nwords        = (length / VALUE_LEN) + ((rem != 0) ? 1 : 0);
            last_mask     = (rem == 0) ? 0 : ((uint64_t(1) << rem) - 1);
            for ( int i = 0; i < MAX_WORDS; ++i ) {
                words[i] = 0;
            }
        }

        uint64_t value() const
        {
            uint64_t joined = 0;
            for ( int i = 0; i < nwords; ++i ) {
                joined ^= words[i];
            }

            uint64_t result = joined;
            for ( int i = 0; i <= WORDS_PER_VALUE; ++i ) {
                joined >>= WORD_LEN;
                result ^= joined;
            }

            return result & ((uint64_t(1) << WORD_LEN) - 1);
        }

        void push_back(const bool ins)
        {
            const uint64_t value_mask = (uint64_t(1) << VALUE_LEN) - 1;

            uint64_t carry = ins ? 1 : 0;
            for ( int i = 0; i < nwords; ++i ) {
                const uint64_t msb = (words[i] >> (VALUE_LEN - 1)) & 1;
                words[i]           = ((words[i] << 1) | carry) & value_mask;
                carry              = msb;
            }

            if ( (last_mask != 0) && (nwords > 0) ) { words[nwords - 1] &= last_mask; }
        }
    };

    struct History {
        FoldedShiftRegister ghist[NTABLES];
        uint64_t            ras[RAS_DEPTH] = {0};
        uint32_t            ras_top        = 0;
    };

    struct Checkpoint {
        // The footprint: which weight each table contributed, and the sum.
        uint32_t indices[NTABLES] = {0};
        int      yout             = 0;

        // The speculative history as it stood before this branch.
        uint64_t s_words[NTABLES][MAX_WORDS] = {{0}};
        uint64_t s_ras[RAS_DEPTH]            = {0};
        uint32_t s_ras_top                   = 0;

        uint64_t           pc          = 0;
        uint64_t           hist_target = 0;
        uint64_t           fallthrough = 0;
        uint64_t           pred_target = 0;
        VanadisBranchClass cls         = VanadisBranchClass::CONDITIONAL;
        bool               pred_dir    = false;
        bool               used_ras    = false;
    };

    VanadisPerceptronCore() { reinit(); }

    void reinit()
    {
        memset(tables, 0, sizeof(tables));
        theta = 10;
        tc    = 0;

        const int* len = historyLengths();
        for ( int i = 0; i < NTABLES; ++i ) {
            arch_.ghist[i].init(len[i]);
        }
        memset(arch_.ras, 0, sizeof(arch_.ras));
        arch_.ras_top = 0;

        spec_ = arch_;
    }

    History&       speculative() { return spec_; }
    const History& architected() const { return arch_; }

    // ---- prediction --------------------------------------------------------
    void snapshot(Checkpoint& C, const uint64_t pc) const
    {
        C.pc = pc;
        for ( int i = 0; i < NTABLES; ++i ) {
            for ( int w = 0; w < MAX_WORDS; ++w ) {
                C.s_words[i][w] = spec_.ghist[i].words[w];
            }
        }
        memcpy(C.s_ras, spec_.ras, sizeof(C.s_ras));
        C.s_ras_top = spec_.ras_top;
    }

    bool predict(Checkpoint& C, const uint64_t pc, const VanadisBranchClass cls, const uint64_t hist_target,
                 const uint64_t fallthrough)
    {
        C.cls         = cls;
        C.hist_target = hist_target;
        C.fallthrough = fallthrough;

        const uint64_t pc_slice = pc & ((uint64_t(1) << TABLE_INDEX_BITS) - 1);

        int sum = 0;
        for ( int i = 0; i < NTABLES; ++i ) {
            C.indices[i] = static_cast<uint32_t>(spec_.ghist[i].value() ^ pc_slice);
            sum += tables[i][C.indices[i]];
        }
        C.yout = sum;

        // The sum decides a conditional branch. An unconditional branch is
        // taken whatever the sum says, but the sum is still computed and still
        // trained on, which is what the reference does.
        const bool dir = vanadisBranchIsConditional(cls) ? (sum >= THRESHOLD) : true;
        C.pred_dir     = dir;

        for ( int i = 0; i < NTABLES; ++i ) {
            spec_.ghist[i].push_back(dir);
        }

        return dir;
    }

    // ---- training ----------------------------------------------------------
    void update(const uint64_t pc, const VanadisBranchClass cls, const bool taken, Checkpoint& C)
    {
        for ( int i = 0; i < NTABLES; ++i ) {
            arch_.ghist[i].push_back(taken);
        }

        const bool prediction_correct = (taken == (C.yout >= THRESHOLD));
        const bool prediction_weak    = (abs(C.yout) < theta);

        if ( !prediction_correct || prediction_weak ) {
            for ( int i = 0; i < NTABLES; ++i ) {
                int w = tables[i][C.indices[i]] + (taken ? 1 : -1);
                if ( w > WEIGHT_MAX ) w = WEIGHT_MAX;
                if ( w < WEIGHT_MIN ) w = WEIGHT_MIN;
                tables[i][C.indices[i]] = static_cast<int8_t>(w);
            }
            adjustThreshold(prediction_correct);
        }

        // The return address stack has an architected copy for the same reason
        // the histories do: it is pushed and popped at prediction time, so
        // discarding everything unretired has to have something to go back to.
        stackAction(arch_, cls, C.fallthrough);
    }

    // ---- repair ------------------------------------------------------------

    // Nothing here is kept per index, so a wrong-path branch leaves nothing
    // behind that the oldest checkpoint does not already hold.
    void restoreWords(const Checkpoint&) {}

    void repair(const Checkpoint& C, const bool taken)
    {
        for ( int i = 0; i < NTABLES; ++i ) {
            for ( int w = 0; w < MAX_WORDS; ++w ) {
                spec_.ghist[i].words[w] = C.s_words[i][w];
            }
        }
        memcpy(spec_.ras, C.s_ras, sizeof(spec_.ras));
        spec_.ras_top = C.s_ras_top;

        for ( int i = 0; i < NTABLES; ++i ) {
            spec_.ghist[i].push_back(taken);
        }

        applyStackAction(C.cls, C.fallthrough);
    }

    void repairToCommit() { spec_ = arch_; }

    // ---- the return address stack -----------------------------------------
    static void stackAction(History& H, const VanadisBranchClass cls, const uint64_t fallthrough)
    {
        if ( vanadisBranchIsCall(cls) ) {
            H.ras[H.ras_top] = fallthrough;
            H.ras_top        = (H.ras_top + 1) % RAS_DEPTH;
        }
        else if ( VanadisBranchClass::RETURN == cls ) {
            H.ras_top = (H.ras_top + RAS_DEPTH - 1) % RAS_DEPTH;
        }
    }

    void applyStackAction(const VanadisBranchClass cls, const uint64_t fallthrough)
    {
        stackAction(spec_, cls, fallthrough);
    }

    void rasPush(const uint64_t addr)
    {
        spec_.ras[spec_.ras_top] = addr;
        spec_.ras_top            = (spec_.ras_top + 1) % RAS_DEPTH;
    }

    uint64_t rasPop()
    {
        spec_.ras_top = (spec_.ras_top + RAS_DEPTH - 1) % RAS_DEPTH;
        return spec_.ras[spec_.ras_top];
    }

    // ---- serialization -----------------------------------------------------
    static void serializeHistory(const History& H, std::vector<uint8_t>& out)
    {
        out.clear();
        for ( int i = 0; i < NTABLES; ++i ) {
            append(out, H.ghist[i].words, sizeof(H.ghist[i].words));
        }
        append(out, H.ras, sizeof(H.ras));
        append(out, &H.ras_top, sizeof(H.ras_top));
    }

    void serializeSpeculative(std::vector<uint8_t>& out) const { serializeHistory(spec_, out); }

    void serializeAll(std::vector<uint8_t>& out) const
    {
        out.clear();
        append(out, tables, sizeof(tables));
        append(out, &theta, sizeof(theta));
        append(out, &tc, sizeof(tc));

        std::vector<uint8_t> h;
        serializeHistory(arch_, h);
        out.insert(out.end(), h.begin(), h.end());
    }

    // The storage the modelled predictor holds, in bits: the weights and the
    // shift registers. The return address stack is not part of it.
    static int storageBits()
    {
        int s = NTABLES * TABLE_SIZE * 8;
        for ( int i = 0; i < NTABLES; ++i ) {
            s += historyLengths()[i];
        }
        return s;
    }

private:
    template <typename T>
    static void append(std::vector<uint8_t>& out, const T* p, size_t n)
    {
        const uint8_t* b = reinterpret_cast<const uint8_t*>(p);
        out.insert(out.end(), b, b + n);
    }

    // The dynamic threshold, from Seznec's O-GEHL predictor.
    void adjustThreshold(const bool correct)
    {
        if ( !correct ) {
            tc++;
            if ( tc >= SPEED ) {
                theta++;
                tc = 0;
            }
        }
        else {
            tc--;
            if ( tc <= -SPEED ) {
                theta--;
                tc = 0;
            }
        }
    }

    static int abs(const int v) { return (v < 0) ? -v : v; }

    int8_t  tables[NTABLES][TABLE_SIZE] = {{0}};
    int     theta                      = 10;
    int     tc                         = 0;
    History spec_;
    History arch_;
};

} // namespace Vanadis
} // namespace SST

#endif
