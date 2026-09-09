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
// TAGE-SC-L: a tagged predictor with geometric history lengths (TAGE), a
// statistical corrector (SC) and a loop predictor (L). The tables, their
// dimensions, their index and tag functions, the allocation policy and the
// update rules are A. Seznec's CBP-2016 64 KiB entry, unchanged.
//
// What is different here is that this core runs in an out-of-order machine.
// The original is written for a driver that predicts one branch and then
// immediately tells the predictor how that branch turned out, so it keeps the
// whole prediction footprint -- table indices, tags, the hitting bank, the
// confidences, the statistical corrector's sum -- in variables shared between
// the two calls, and it advances the history inside the update, in program
// order. An out-of-order front end has many predictions outstanding at once
// and predicts down paths that turn out not to be executed, so:
//
//   * every one of those shared variables lives in a per-branch record
//     (`Checkpoint`), taken when the branch is predicted and read back when it
//     retires;
//
//   * the history exists twice, as a speculative copy advanced at prediction
//     time so the next prediction sees this branch, and an architected copy
//     advanced at retirement, which is the one a repair restores from;
//
//   * a misprediction restores the speculative copy from the record of the
//     branch that mispredicted and replays that branch's own history update
//     with the direction and target that actually happened.
//
// The tables themselves are never speculative: they are written only at
// retirement, in program order, exactly as in the original.
// ---------------------------------------------------------------------------

#ifndef _H_VANADIS_BRANCH_UNIT_TAGE_SCL
#define _H_VANADIS_BRANCH_UNIT_TAGE_SCL

#include "vbranch/vbranchcheckpoint.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace SST {
namespace Vanadis {

// The index expressions are written as macros because they read a dozen names
// from the surrounding scope, exactly as in the original. Every one of those
// names is either the function's `PC` argument or a field of the per-branch
// record bound to a reference of the same name, so an expression here computes
// the same index at retirement that it computed at prediction. They are
// undefined again at the end of this file.
#define VTAGE_INDBIAS \
    (((((PC ^ (PC >> 2)) << 1) ^ (LowConf & (LongestMatchPred != alttaken))) << 1) + pred_inter) & ((1 << LOGBIAS) - 1)
#define VTAGE_INDBIASSK (((((PC ^ (PC >> (LOGBIAS - 2))) << 1) ^ (HighConf)) << 1) + pred_inter) & ((1 << LOGBIAS) - 1)
#define VTAGE_INDBIASBANK                                                                                       \
    (pred_inter + (((HitBank + 1) / 4) << 4) + (HighConf << 1) + (LowConf << 2) + ((AltBank != 0) << 3) +        \
     ((PC ^ (PC >> 2)) << 7)) &                                                                                 \
        ((1 << LOGBIAS) - 1)
#define VTAGE_INDLOCAL  ((PC ^ (PC >> 2)) & (NLOCAL - 1))
#define VTAGE_INDSLOCAL (((PC ^ (PC >> 5))) & (NSECLOCAL - 1))
#define VTAGE_INDTLOCAL (((PC ^ (PC >> (LOGTNB)))) & (NTLOCAL - 1))
#define VTAGE_INDUPD    (PC ^ (PC >> 2)) & ((1 << LOGSIZEUP) - 1)
#define VTAGE_INDUPDS   ((PC ^ (PC >> 2)) & ((1 << (LOGSIZEUPS)) - 1))
#define VTAGE_INDUSEALT (((((HitBank - 1) / 8) << 1) + AltConf) % (SIZEUSEALT - 1))
#define VTAGE_GINDEX                                                                                            \
    (((long long)PC) ^ bhist ^ (bhist >> (8 - i)) ^ (bhist >> (16 - 2 * i)) ^ (bhist >> (24 - 3 * i)) ^          \
     (bhist >> (32 - 3 * i)) ^ (bhist >> (40 - 4 * i))) &                                                       \
        ((1 << (logs - (i >= (NBR - 2)))) - 1)

class VanadisTageSclCore
{
public:
    // ---- geometry, all from the reference's 64 KiB entry -------------------
    enum : int {
        BORNTICK        = 1024,
        PERCWIDTH       = 6,
        LOGBIAS         = 8,
        LOGINB          = 8,
        INB             = 1,
        LOGIMNB         = 9,
        IMNB            = 2,
        LOGGNB          = 10,
        GNB             = 3,
        PNB             = 3,
        LOGPNB          = 9,
        LOGLNB          = 10,
        LNB             = 3,
        LOGLOCAL        = 8,
        NLOCAL          = (1 << LOGLOCAL),
        LOGSNB          = 9,
        SNB             = 3,
        LOGSECLOCAL     = 4,
        NSECLOCAL       = (1 << LOGSECLOCAL),
        LOGTNB          = 10,
        TNB             = 2,
        NTLOCAL         = 16,
        WIDTHRES        = 12,
        WIDTHRESP       = 8,
        LOGSIZEUP       = 6,
        LOGSIZEUPS      = (LOGSIZEUP / 2),
        EWIDTH          = 6,
        CONFWIDTH       = 7,
        HISTBUFFERLENGTH= 4096,
        NHIST           = 36,
        NBANKLOW        = 10,
        NBANKHIGH       = 20,
        BORN            = 13,
        BORNINFASSOC    = 9,
        BORNSUPASSOC    = 23,
        MINHIST         = 6,
        MAXHIST         = 3000,
        LOGG            = 10,
        TBITS           = 8,
        NNN             = 1,
        HYSTSHIFT       = 2,
        LOGB            = 13,
        PHISTWIDTH      = 27,
        UWIDTH          = 1,
        CWIDTH          = 3,
        LOGSIZEUSEALT   = 4,
        ALTWIDTH        = 5,
        SIZEUSEALT      = (1 << LOGSIZEUSEALT),
        LOGL            = 5,
        WIDTHNBITERLOOP = 10,
        LOOPTAG         = 10,
        CONFLOOP        = 15,
        RAS_DEPTH       = 32
    };

    // The cyclic shift register that folds a long history down to the number
    // of bits an index needs. P. Michaud's, from CBP-1.
    class folded_history
    {
    public:
        unsigned comp     = 0;
        int      CLENGTH  = 0;
        int      OLENGTH  = 0;
        int      OUTPOINT = 0;

        void init(int original_length, int compressed_length)
        {
            comp     = 0;
            OLENGTH  = original_length;
            CLENGTH  = compressed_length;
            OUTPOINT = OLENGTH % CLENGTH;
        }

        void update(const uint8_t* h, int PT)
        {
            comp = (comp << 1) ^ h[PT & (HISTBUFFERLENGTH - 1)];
            comp ^= h[(PT + OLENGTH) & (HISTBUFFERLENGTH - 1)] << OUTPOINT;
            comp ^= (comp >> CLENGTH);
            comp = (comp) & ((1 << CLENGTH) - 1);
        }
    };

    class bentry // bimodal table entry
    {
    public:
        int8_t hyst = 1;
        int8_t pred = 0;
    };

    class gentry // tagged table entry
    {
    public:
        int8_t   ctr = 0;
        uint32_t tag = 0;
        int8_t   u   = 0;
    };

    class lentry // loop predictor entry
    {
    public:
        uint16_t NbIter      = 0;
        uint8_t  confid      = 0;
        uint16_t CurrentIter = 0;
        uint16_t TAG         = 0;
        uint8_t  age         = 0;
        bool     dir         = false;
    };

    // Everything a branch advances when it is predicted, and therefore
    // everything a repair has to be able to put back. There are two of these:
    // the speculative copy the predictions read, and the architected copy the
    // retirements advance.
    struct History {
        long long      GHIST   = 0;
        long long      phist   = 0;
        int            ptghist = 0;
        long long      IMLIcount = 0;
        folded_history ch_i[NHIST + 1];
        folded_history ch_t[2][NHIST + 1];
        long long      L_shist[NLOCAL]   = {0};
        long long      S_slhist[NSECLOCAL] = {0};
        long long      T_slhist[NTLOCAL]  = {0};
        long long      IMHIST[256]        = {0};
        uint8_t        ghist[HISTBUFFERLENGTH] = {0};

        // The return address stack is speculative for the same reason: it is
        // pushed and popped at prediction time.
        uint64_t ras[RAS_DEPTH] = {0};
        uint32_t ras_top        = 0;
    };

    // The per-branch record. Holds the prediction footprint the training needs
    // and enough of the speculative history to undo this branch and everything
    // fetched after it.
    struct Checkpoint {
        // -- the footprint the training consumes ----------------------------
        int      GI[NHIST + 1]   = {0};
        uint32_t GTAG[NHIST + 1] = {0};
        int      BI              = 0;
        int8_t   BIM             = 0;
        int      HitBank         = 0;
        int      AltBank         = 0;
        bool     LongestMatchPred = false;
        bool     alttaken         = false;
        bool     tage_pred        = false;
        bool     pred_taken       = false;
        bool     pred_inter       = false;
        bool     HighConf         = false;
        bool     MedConf          = false;
        bool     LowConf          = false;
        bool     AltConf          = false;
        int      LSUM             = 0;
        int      THRES            = 0;
        bool     predloop         = false;
        bool     LVALID           = false;
        int      LHIT             = -1;
        int      LI               = 0;
        int      LIB              = 0;
        int      LTAG             = 0;

        // -- the speculative history as it stood before this branch ---------
        long long s_GHIST     = 0;
        long long s_phist     = 0;
        int       s_ptghist   = 0;
        long long s_IMLIcount = 0;
        unsigned  s_ch_i[NHIST + 1]  = {0};
        unsigned  s_ch_t0[NHIST + 1] = {0};
        unsigned  s_ch_t1[NHIST + 1] = {0};

        // Only the words this branch can touch are kept, with the index that
        // says where they go back. Restoring the wrong-path branches newest
        // first puts every word that moved back where it was.
        int       idx_local  = 0;
        int       idx_slocal = 0;
        int       idx_tlocal = 0;
        int       idx_imhist = 0;
        long long s_L_shist  = 0;
        long long s_S_slhist = 0;
        long long s_T_slhist = 0;
        long long s_IMHIST   = 0;

        uint64_t s_ras[RAS_DEPTH] = {0};
        uint32_t s_ras_top        = 0;

        // -- what this branch was predicted to do ---------------------------
        uint64_t           pc          = 0;
        uint64_t           hist_target = 0;
        uint64_t           fallthrough = 0;
        uint64_t           pred_target = 0;
        VanadisBranchClass cls         = VanadisBranchClass::CONDITIONAL;
        bool               pred_dir    = false;
        bool               used_ras    = false;
    };

    VanadisTageSclCore() { reinit(); }

    ~VanadisTageSclCore()
    {
        delete[] btable;
        delete[] gtable[1];
        delete[] gtable[BORN];
        delete[] ltable;
    }

    VanadisTageSclCore(const VanadisTageSclCore&)            = delete;
    VanadisTageSclCore& operator=(const VanadisTageSclCore&) = delete;

    // ---- construction ------------------------------------------------------
    void reinit()
    {
        m[1]         = MINHIST;
        m[NHIST / 2] = MAXHIST;
        for ( int i = 2; i <= NHIST / 2; i++ ) {
            m[i] = (int)(((double)MINHIST *
                          pow((double)(MAXHIST) / (double)MINHIST, (double)(i - 1) / (double)(((NHIST / 2) - 1)))) +
                         0.5);
        }
        for ( int i = 1; i <= NHIST; i++ ) {
            NOSKIP[i] = ((i - 1) & 1) || ((i >= BORNINFASSOC) & (i < BORNSUPASSOC));
        }

        NOSKIP[4]         = 0;
        NOSKIP[NHIST - 2] = 0;
        NOSKIP[8]         = 0;
        NOSKIP[NHIST - 6] = 0;

        for ( int i = NHIST; i > 1; i-- ) {
            m[i] = m[(i + 1) / 2];
        }
        for ( int i = 1; i <= NHIST; i++ ) {
            TB[i]   = TBITS + 4 * (i >= BORN);
            logg[i] = LOGG;
        }

        ltable = new lentry[1 << LOGL];

        gtable[1]    = new gentry[NBANKLOW * (1 << LOGG)];
        SizeTable[1] = NBANKLOW * (1 << LOGG);

        gtable[BORN]    = new gentry[NBANKHIGH * (1 << LOGG)];
        SizeTable[BORN] = NBANKHIGH * (1 << LOGG);

        for ( int i = BORN + 1; i <= NHIST; i++ )
            gtable[i] = gtable[BORN];
        for ( int i = 2; i <= BORN - 1; i++ )
            gtable[i] = gtable[1];
        btable = new bentry[1 << LOGB];

        for ( int i = 1; i <= NHIST; i++ ) {
            arch_.ch_i[i].init(m[i], (logg[i]));
            arch_.ch_t[0][i].init(arch_.ch_i[i].OLENGTH, TB[i]);
            arch_.ch_t[1][i].init(arch_.ch_i[i].OLENGTH, TB[i] - 1);
        }

        WITHLOOP        = -1;
        Seed            = 0;
        TICK            = 0;
        updatethreshold = 35 << 3;

        memset(Pupdatethreshold, 0, sizeof(Pupdatethreshold));
        memset(GGEHLA, 0, sizeof(GGEHLA));
        memset(LGEHLA, 0, sizeof(LGEHLA));
        memset(SGEHLA, 0, sizeof(SGEHLA));
        memset(TGEHLA, 0, sizeof(TGEHLA));
        memset(PGEHLA, 0, sizeof(PGEHLA));
        memset(IGEHLA, 0, sizeof(IGEHLA));
        memset(IMGEHLA, 0, sizeof(IMGEHLA));

        for ( int i = 0; i < GNB; i++ )
            GGEHL[i] = &GGEHLA[i][0];
        for ( int i = 0; i < LNB; i++ )
            LGEHL[i] = &LGEHLA[i][0];
        for ( int i = 0; i < SNB; i++ )
            SGEHL[i] = &SGEHLA[i][0];
        for ( int i = 0; i < TNB; i++ )
            TGEHL[i] = &TGEHLA[i][0];
        for ( int i = 0; i < PNB; i++ )
            PGEHL[i] = &PGEHLA[i][0];
        for ( int i = 0; i < INB; i++ )
            IGEHL[i] = &IGEHLA[i][0];
        for ( int i = 0; i < IMNB; i++ )
            IMGEHL[i] = &IMGEHLA[i][0];

        for ( int i = 0; i < GNB; i++ )
            for ( int j = 0; j < ((1 << LOGGNB) - 1); j++ )
                if ( !(j & 1) ) GGEHL[i][j] = -1;
        for ( int i = 0; i < LNB; i++ )
            for ( int j = 0; j < ((1 << LOGLNB) - 1); j++ )
                if ( !(j & 1) ) LGEHL[i][j] = -1;
        for ( int i = 0; i < INB; i++ )
            for ( int j = 0; j < ((1 << LOGINB) - 1); j++ )
                if ( !(j & 1) ) IGEHL[i][j] = -1;
        for ( int i = 0; i < IMNB; i++ )
            for ( int j = 0; j < ((1 << LOGIMNB) - 1); j++ )
                if ( !(j & 1) ) IMGEHL[i][j] = -1;
        for ( int i = 0; i < SNB; i++ )
            for ( int j = 0; j < ((1 << LOGSNB) - 1); j++ )
                if ( !(j & 1) ) SGEHL[i][j] = -1;
        for ( int i = 0; i < TNB; i++ )
            for ( int j = 0; j < ((1 << LOGTNB) - 1); j++ )
                if ( !(j & 1) ) TGEHL[i][j] = -1;
        for ( int i = 0; i < PNB; i++ )
            for ( int j = 0; j < ((1 << LOGPNB) - 1); j++ )
                if ( !(j & 1) ) PGEHL[i][j] = -1;

        for ( int i = 0; i < (1 << LOGB); i++ ) {
            btable[i].pred = 0;
            btable[i].hyst = 1;
        }

        for ( int j = 0; j < (1 << LOGBIAS); j++ ) {
            switch ( j & 3 ) {
            case 0: BiasSK[j] = -8; break;
            case 1: BiasSK[j] = 7; break;
            case 2: BiasSK[j] = -32; break;
            case 3: BiasSK[j] = 31; break;
            }
        }
        for ( int j = 0; j < (1 << LOGBIAS); j++ ) {
            switch ( j & 3 ) {
            case 0: Bias[j] = -32; break;
            case 1: Bias[j] = 31; break;
            case 2: Bias[j] = -1; break;
            case 3: Bias[j] = 0; break;
            }
        }
        for ( int j = 0; j < (1 << LOGBIAS); j++ ) {
            switch ( j & 3 ) {
            case 0: BiasBank[j] = -32; break;
            case 1: BiasBank[j] = 31; break;
            case 2: BiasBank[j] = -1; break;
            case 3: BiasBank[j] = 0; break;
            }
        }

        for ( int i = 0; i < SIZEUSEALT; i++ )
            use_alt_on_na[i] = 0;

        memset(WG, 0, sizeof(WG));
        memset(WL, 0, sizeof(WL));
        memset(WS, 0, sizeof(WS));
        memset(WT, 0, sizeof(WT));
        memset(WP, 0, sizeof(WP));
        memset(WI, 0, sizeof(WI));
        memset(WIM, 0, sizeof(WIM));
        memset(WB, 0, sizeof(WB));
        for ( int i = 0; i < (1 << LOGSIZEUPS); i++ ) {
            WG[i] = 7;
            WL[i] = 7;
            WS[i] = 7;
            WT[i] = 7;
            WP[i] = 7;
            WI[i] = 7;
            WB[i] = 4;
        }

        FirstH  = 0;
        SecondH = 0;

        // The speculative copy starts as a copy of the architected one, which
        // is the whole of the invariant the repair maintains.
        copyHistory(arch_, spec_);
    }

    History&       speculative() { return spec_; }
    const History& speculative() const { return spec_; }
    const History& architected() const { return arch_; }

    // ---- prediction --------------------------------------------------------

    // Records the speculative state this branch is about to change, so that a
    // repair can put it back.
    void snapshot(Checkpoint& C, const uint64_t PC) const
    {
        C.pc          = PC;
        C.s_GHIST     = spec_.GHIST;
        C.s_phist     = spec_.phist;
        C.s_ptghist   = spec_.ptghist;
        C.s_IMLIcount = spec_.IMLIcount;

        for ( int i = 0; i <= NHIST; i++ ) {
            C.s_ch_i[i]  = spec_.ch_i[i].comp;
            C.s_ch_t0[i] = spec_.ch_t[0][i].comp;
            C.s_ch_t1[i] = spec_.ch_t[1][i].comp;
        }

        C.idx_local  = VTAGE_INDLOCAL;
        C.idx_slocal = VTAGE_INDSLOCAL;
        C.idx_tlocal = VTAGE_INDTLOCAL;
        C.idx_imhist = (int)(spec_.IMLIcount & 255);

        C.s_L_shist  = spec_.L_shist[C.idx_local];
        C.s_S_slhist = spec_.S_slhist[C.idx_slocal];
        C.s_T_slhist = spec_.T_slhist[C.idx_tlocal];
        C.s_IMHIST   = spec_.IMHIST[C.idx_imhist];

        memcpy(C.s_ras, spec_.ras, sizeof(C.s_ras));
        C.s_ras_top = spec_.ras_top;
    }

    // The direction, read from the speculative history. Unconditional classes
    // do not consult the tables; they still advance the history, because the
    // history is a record of every branch.
    bool predict(Checkpoint& C, const uint64_t PC, const VanadisBranchClass cls, const uint64_t hist_target,
                 const uint64_t fallthrough)
    {
        C.cls         = cls;
        C.hist_target = hist_target;
        C.fallthrough = fallthrough;

        bool dir = true;
        if ( vanadisBranchIsConditional(cls) ) { dir = GetPrediction(PC, C); }
        else {
            C.pred_taken = true;
            C.tage_pred  = true;
        }

        C.pred_dir = dir;

        // The speculative history moves now, so that the next prediction sees
        // this branch. It moves with the predicted direction and target,
        // because that is all that is known yet.
        HistoryUpdate(spec_, PC, cls, dir, hist_target);

        return dir;
    }

    // ---- training, at retirement, in program order -------------------------
    void update(const uint64_t PC, const VanadisBranchClass cls, const bool taken, Checkpoint& C)
    {
        if ( vanadisBranchIsConditional(cls) ) { UpdatePredictor(PC, taken, C); }
        else {
            HistoryUpdate(arch_, PC, cls, taken, C.hist_target);
        }

        // The return address stack has an architected copy for the same reason
        // the histories do: it is pushed and popped at prediction time, so
        // discarding everything unretired has to have something to go back to.
        stackAction(arch_, cls, C.fallthrough);
    }

    // ---- repair ------------------------------------------------------------

    // Put back one branch's word-level history. Called newest first, so that
    // every word a wrong-path branch moved ends at the value the oldest of
    // them found.
    void restoreWords(const Checkpoint& C)
    {
        spec_.L_shist[C.idx_local]   = C.s_L_shist;
        spec_.S_slhist[C.idx_slocal] = C.s_S_slhist;
        spec_.T_slhist[C.idx_tlocal] = C.s_T_slhist;
        spec_.IMHIST[C.idx_imhist]   = C.s_IMHIST;
    }

    // Put back the state that is a single copy, then replay this branch with
    // what actually happened.
    void repair(const Checkpoint& C, const bool taken)
    {
        restoreWords(C);

        spec_.GHIST     = C.s_GHIST;
        spec_.phist     = C.s_phist;
        spec_.ptghist   = C.s_ptghist;
        spec_.IMLIcount = C.s_IMLIcount;

        for ( int i = 0; i <= NHIST; i++ ) {
            spec_.ch_i[i].comp    = C.s_ch_i[i];
            spec_.ch_t[0][i].comp = C.s_ch_t0[i];
            spec_.ch_t[1][i].comp = C.s_ch_t1[i];
        }

        memcpy(spec_.ras, C.s_ras, sizeof(spec_.ras));
        spec_.ras_top = C.s_ras_top;

        // The bit buffer the folded registers read from does not have to be
        // restored: every slot a squashed branch wrote is written again, in
        // the same order, before anything reads it. The write pointer and the
        // folded registers are what carry the state, and both are back.
        HistoryUpdate(spec_, C.pc, C.cls, taken, C.hist_target);

        applyStackAction(C.cls, C.fallthrough);
    }

    // Everything unretired is gone: the architected state is the right
    // speculative state.
    void repairToCommit() { copyHistory(arch_, spec_); }

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

    uint64_t rasPeekNext() const { return spec_.ras[(spec_.ras_top + RAS_DEPTH - 1) % RAS_DEPTH]; }

    // ---- serialization, for the trace-driven tests -------------------------
    void serializeTables(std::vector<uint8_t>& out) const
    {
        out.clear();
        append(out, Bias, sizeof(Bias));
        append(out, BiasSK, sizeof(BiasSK));
        append(out, BiasBank, sizeof(BiasBank));
        append(out, GGEHLA, sizeof(GGEHLA));
        append(out, PGEHLA, sizeof(PGEHLA));
        append(out, LGEHLA, sizeof(LGEHLA));
        append(out, SGEHLA, sizeof(SGEHLA));
        append(out, TGEHLA, sizeof(TGEHLA));
        append(out, IGEHLA, sizeof(IGEHLA));
        append(out, IMGEHLA, sizeof(IMGEHLA));
        append(out, WG, sizeof(WG));
        append(out, WL, sizeof(WL));
        append(out, WS, sizeof(WS));
        append(out, WT, sizeof(WT));
        append(out, WP, sizeof(WP));
        append(out, WI, sizeof(WI));
        append(out, WIM, sizeof(WIM));
        append(out, WB, sizeof(WB));
        append(out, &updatethreshold, sizeof(updatethreshold));
        append(out, Pupdatethreshold, sizeof(Pupdatethreshold));
        append(out, &FirstH, sizeof(FirstH));
        append(out, &SecondH, sizeof(SecondH));
        append(out, use_alt_on_na, sizeof(use_alt_on_na));
        append(out, &TICK, sizeof(TICK));
        append(out, &Seed, sizeof(Seed));
        append(out, &WITHLOOP, sizeof(WITHLOOP));
        // Field by field, because the padding inside these entries is never
        // written and comparing it would compare uninitialised bytes.
        for ( int i = 0; i < (1 << LOGB); i++ ) {
            append(out, &btable[i].pred, sizeof(int8_t));
            append(out, &btable[i].hyst, sizeof(int8_t));
        }
        for ( int bank = 0; bank < 2; bank++ ) {
            const int     which = (bank == 0) ? 1 : BORN;
            const gentry* g     = gtable[which];
            for ( int i = 0; i < SizeTable[which]; i++ ) {
                append(out, &g[i].ctr, sizeof(int8_t));
                append(out, &g[i].tag, sizeof(uint32_t));
                append(out, &g[i].u, sizeof(int8_t));
            }
        }
        for ( int i = 0; i < (1 << LOGL); i++ ) {
            append(out, &ltable[i].NbIter, sizeof(uint16_t));
            append(out, &ltable[i].confid, sizeof(uint8_t));
            append(out, &ltable[i].CurrentIter, sizeof(uint16_t));
            append(out, &ltable[i].TAG, sizeof(uint16_t));
            append(out, &ltable[i].age, sizeof(uint8_t));
            append(out, &ltable[i].dir, sizeof(bool));
        }
    }

    static void serializeHistory(const History& H, std::vector<uint8_t>& out)
    {
        out.clear();
        append(out, &H.GHIST, sizeof(H.GHIST));
        append(out, &H.phist, sizeof(H.phist));
        append(out, &H.ptghist, sizeof(H.ptghist));
        append(out, &H.IMLIcount, sizeof(H.IMLIcount));
        for ( int i = 0; i <= NHIST; i++ ) {
            append(out, &H.ch_i[i].comp, sizeof(unsigned));
            append(out, &H.ch_t[0][i].comp, sizeof(unsigned));
            append(out, &H.ch_t[1][i].comp, sizeof(unsigned));
        }
        append(out, H.L_shist, sizeof(H.L_shist));
        append(out, H.S_slhist, sizeof(H.S_slhist));
        append(out, H.T_slhist, sizeof(H.T_slhist));
        append(out, H.IMHIST, sizeof(H.IMHIST));
        append(out, H.ras, sizeof(H.ras));
        append(out, &H.ras_top, sizeof(H.ras_top));
    }

    void serializeSpeculative(std::vector<uint8_t>& out) const { serializeHistory(spec_, out); }

    void serializeAll(std::vector<uint8_t>& out) const
    {
        std::vector<uint8_t> t;
        serializeTables(t);
        std::vector<uint8_t> h;
        serializeHistory(arch_, h);
        out = t;
        out.insert(out.end(), h.begin(), h.end());
    }

    // The storage the modelled predictor holds, in bits, by the reference's
    // own accounting. The return address stack is not part of it.
    static int storageBits()
    {
        int s = 0;
        s += NBANKHIGH * (1 << LOGG) * (CWIDTH + UWIDTH + (TBITS + 4));
        s += NBANKLOW * (1 << LOGG) * (CWIDTH + UWIDTH + TBITS);
        s += SIZEUSEALT * ALTWIDTH;
        s += (1 << LOGB) + (1 << (LOGB - HYSTSHIFT));
        s += MAXHIST;
        s += PHISTWIDTH;
        s += 10;
        s += (1 << LOGL) * (2 * WIDTHNBITERLOOP + LOOPTAG + 4 + 4 + 1);
        s += WIDTHRESP * (1 << LOGSIZEUP);
        s += 3 * EWIDTH * (1 << LOGSIZEUPS);
        s += PERCWIDTH * 3 * (1 << LOGBIAS);
        s += (GNB - 2) * (1 << LOGGNB) * PERCWIDTH + (1 << (LOGGNB - 1)) * (2 * PERCWIDTH) + 40;
        s += (PNB - 2) * (1 << LOGPNB) * PERCWIDTH + (1 << (LOGPNB - 1)) * (2 * PERCWIDTH);
        s += (LNB - 2) * (1 << LOGLNB) * PERCWIDTH + (1 << (LOGLNB - 1)) * (2 * PERCWIDTH) + NLOCAL * 11 +
             EWIDTH * (1 << LOGSIZEUPS);
        s += (SNB - 2) * (1 << LOGSNB) * PERCWIDTH + (1 << (LOGSNB - 1)) * (2 * PERCWIDTH) + NSECLOCAL * 16 +
             EWIDTH * (1 << LOGSIZEUPS);
        s += (TNB - 2) * (1 << LOGTNB) * PERCWIDTH + (1 << (LOGTNB - 1)) * (2 * PERCWIDTH) + NTLOCAL * 9 +
             EWIDTH * (1 << LOGSIZEUPS);
        s += (1 << (LOGINB - 1)) * PERCWIDTH + 8;
        s += IMNB * (1 << (LOGIMNB - 1)) * PERCWIDTH + 2 * EWIDTH * (1 << LOGSIZEUPS) + 256 * 10;
        s += 2 * CONFWIDTH;
        return s;
    }

private:
    template <typename T>
    static void append(std::vector<uint8_t>& out, const T* p, size_t n)
    {
        const uint8_t* b = reinterpret_cast<const uint8_t*>(p);
        out.insert(out.end(), b, b + n);
    }

    static void copyHistory(const History& from, History& to)
    {
        to.GHIST     = from.GHIST;
        to.phist     = from.phist;
        to.ptghist   = from.ptghist;
        to.IMLIcount = from.IMLIcount;
        for ( int i = 0; i <= NHIST; i++ ) {
            to.ch_i[i]    = from.ch_i[i];
            to.ch_t[0][i] = from.ch_t[0][i];
            to.ch_t[1][i] = from.ch_t[1][i];
        }
        memcpy(to.L_shist, from.L_shist, sizeof(to.L_shist));
        memcpy(to.S_slhist, from.S_slhist, sizeof(to.S_slhist));
        memcpy(to.T_slhist, from.T_slhist, sizeof(to.T_slhist));
        memcpy(to.IMHIST, from.IMHIST, sizeof(to.IMHIST));
        memcpy(to.ghist, from.ghist, sizeof(to.ghist));
        memcpy(to.ras, from.ras, sizeof(to.ras));
        to.ras_top = from.ras_top;
    }

    // ---- the reference's index and update primitives -----------------------

    int F(long long A, int size, int bank) const
    {
        int A1, A2;
        A  = A & ((1 << size) - 1);
        A1 = (A & ((1 << logg[bank]) - 1));
        A2 = (A >> logg[bank]);

        if ( bank < logg[bank] ) A2 = ((A2 << bank) & ((1 << logg[bank]) - 1)) + (A2 >> (logg[bank] - bank));
        A = A1 ^ A2;
        if ( bank < logg[bank] ) A = ((A << bank) & ((1 << logg[bank]) - 1)) + (A >> (logg[bank] - bank));
        return (A);
    }

    int gindex(unsigned int PC, int bank, long long hist, const folded_history* ch_i_aux) const
    {
        int index;
        int M = (m[bank] > PHISTWIDTH) ? PHISTWIDTH : m[bank];
        index = PC ^ (PC >> (abs(logg[bank] - bank) + 1)) ^ ch_i_aux[bank].comp ^ F(hist, M, bank);
        return (index & ((1 << (logg[bank])) - 1));
    }

    uint16_t gtag(unsigned int PC, int bank, const folded_history* ch0, const folded_history* ch1) const
    {
        int tag = (PC) ^ ch0[bank].comp ^ (ch1[bank].comp << 1);
        return (tag & ((1 << (TB[bank])) - 1));
    }

    static void ctrupdate(int8_t& ctr, bool taken, int nbits)
    {
        if ( taken ) {
            if ( ctr < ((1 << (nbits - 1)) - 1) ) ctr++;
        }
        else {
            if ( ctr > -(1 << (nbits - 1)) ) ctr--;
        }
    }

    bool getbim(Checkpoint& C)
    {
        C.BIM      = (btable[C.BI].pred << 1) + (btable[C.BI >> HYSTSHIFT].hyst);
        C.HighConf = (C.BIM == 0) || (C.BIM == 3);
        C.LowConf  = !C.HighConf;
        C.AltConf  = C.HighConf;
        C.MedConf  = false;
        return (btable[C.BI].pred > 0);
    }

    void baseupdate(bool Taken, const Checkpoint& C)
    {
        int inter = C.BIM;
        if ( Taken ) {
            if ( inter < 3 ) inter += 1;
        }
        else if ( inter > 0 )
            inter--;
        btable[C.BI].pred              = inter >> 1;
        btable[C.BI >> HYSTSHIFT].hyst = (inter & 1);
    }

    // The pseudo-random source. It reads the architected history, because it
    // is only ever called from the training, which runs in program order.
    int MYRANDOM()
    {
        Seed++;
        Seed ^= arch_.phist;
        Seed = (Seed >> 21) + (Seed << 11);
        Seed ^= arch_.ptghist;
        Seed = (Seed >> 10) + (Seed << 22);
        return (Seed);
    }

    // ---- TAGE lookup -------------------------------------------------------
    void Tagepred(const uint64_t PC, Checkpoint& C)
    {
        int*       GI   = C.GI;
        uint32_t*  GTAG = C.GTAG;
        const History& H = spec_;

        C.HitBank = 0;
        C.AltBank = 0;
        for ( int i = 1; i <= NHIST; i += 2 ) {
            GI[i]       = gindex(PC, i, H.phist, H.ch_i);
            GTAG[i]     = gtag(PC, i, H.ch_t[0], H.ch_t[1]);
            GTAG[i + 1] = GTAG[i];
            GI[i + 1]   = GI[i] ^ (GTAG[i] & ((1 << LOGG) - 1));
        }
        int T = (PC ^ (H.phist & ((1 << m[BORN]) - 1))) % NBANKHIGH;
        for ( int i = BORN; i <= NHIST; i++ )
            if ( NOSKIP[i] ) {
                GI[i] += (T << LOGG);
                T++;
                T = T % NBANKHIGH;
            }
        T = (PC ^ (H.phist & ((1 << m[1]) - 1))) % NBANKLOW;

        for ( int i = 1; i <= BORN - 1; i++ )
            if ( NOSKIP[i] ) {
                GI[i] += (T << LOGG);
                T++;
                T = T % NBANKLOW;
            }
        C.BI = (PC ^ (PC >> 2)) & ((1 << LOGB) - 1);

        C.alttaken         = getbim(C);
        C.tage_pred        = C.alttaken;
        C.LongestMatchPred = C.alttaken;

        for ( int i = NHIST; i > 0; i-- ) {
            if ( NOSKIP[i] )
                if ( gtable[i][GI[i]].tag == GTAG[i] ) {
                    C.HitBank          = i;
                    C.LongestMatchPred = (gtable[C.HitBank][GI[C.HitBank]].ctr >= 0);
                    break;
                }
        }

        for ( int i = C.HitBank - 1; i > 0; i-- ) {
            if ( NOSKIP[i] )
                if ( gtable[i][GI[i]].tag == GTAG[i] ) {
                    C.AltBank = i;
                    break;
                }
        }

        if ( C.HitBank > 0 ) {
            const int HitBank = C.HitBank;
            if ( C.AltBank > 0 ) {
                C.alttaken = (gtable[C.AltBank][GI[C.AltBank]].ctr >= 0);
                C.AltConf  = (abs(2 * gtable[C.AltBank][GI[C.AltBank]].ctr + 1) > 1);
            }
            else
                C.alttaken = getbim(C);

            const bool AltConf         = C.AltConf;
            bool       Huse_alt_on_na  = (use_alt_on_na[VTAGE_INDUSEALT] >= 0);
            if ( (!Huse_alt_on_na) || (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) > 1) )
                C.tage_pred = C.LongestMatchPred;
            else
                C.tage_pred = C.alttaken;

            C.HighConf = (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) >= (1 << CWIDTH) - 1);
            C.LowConf  = (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) == 1);
            C.MedConf  = (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) == 5);
        }
    }

    int Gpredict(uint64_t PC, long long BHIST, const int* length, int8_t** tab, int NBR, int logs, const int8_t* W)
    {
        int PERCSUM = 0;
        for ( int i = 0; i < NBR; i++ ) {
            long long bhist = BHIST & ((long long)((1 << length[i]) - 1));
            long long index = VTAGE_GINDEX;
            int8_t    ctr   = tab[i][index];
            PERCSUM += (2 * ctr + 1);
        }
        PERCSUM = (1 + (W[VTAGE_INDUPDS] >= 0)) * PERCSUM;
        return PERCSUM;
    }

    void Gupdate(uint64_t PC, bool taken, long long BHIST, const int* length, int8_t** tab, int NBR, int logs,
                 int8_t* W, int LSUM)
    {
        int PERCSUM = 0;

        for ( int i = 0; i < NBR; i++ ) {
            long long bhist = BHIST & ((long long)((1 << length[i]) - 1));
            long long index = VTAGE_GINDEX;

            PERCSUM += (2 * tab[i][index] + 1);
            ctrupdate(tab[i][index], taken, PERCWIDTH);
        }
        {
            int XSUM = LSUM - ((W[VTAGE_INDUPDS] >= 0)) * PERCSUM;
            if ( (XSUM + PERCSUM >= 0) != (XSUM >= 0) )
                ctrupdate(W[VTAGE_INDUPDS], ((PERCSUM >= 0) == taken), EWIDTH);
        }
    }

    // ---- the full prediction ----------------------------------------------
    bool GetPrediction(const uint64_t PC, Checkpoint& C)
    {
        Tagepred(PC, C);
        C.pred_taken = C.tage_pred;

        C.predloop   = getloop(PC, C);
        C.pred_taken = ((WITHLOOP >= 0) && (C.LVALID)) ? C.predloop : C.pred_taken;
        C.pred_inter = C.pred_taken;

        // Names the index macros read.
        const bool LowConf          = C.LowConf;
        const bool HighConf         = C.HighConf;
        const bool LongestMatchPred = C.LongestMatchPred;
        const bool alttaken         = C.alttaken;
        const bool pred_inter       = C.pred_inter;
        const int  HitBank          = C.HitBank;
        const int  AltBank          = C.AltBank;

        int LSUM = 0;

        int8_t ctr = Bias[VTAGE_INDBIAS];
        LSUM += (2 * ctr + 1);
        ctr = BiasSK[VTAGE_INDBIASSK];
        LSUM += (2 * ctr + 1);
        ctr = BiasBank[VTAGE_INDBIASBANK];
        LSUM += (2 * ctr + 1);
        LSUM = (1 + (WB[VTAGE_INDUPDS] >= 0)) * LSUM;

        LSUM += Gpredict((PC << 1) + pred_inter, spec_.GHIST, Gm, GGEHL, GNB, LOGGNB, WG);
        LSUM += Gpredict(PC, spec_.phist, Pm, PGEHL, PNB, LOGPNB, WP);
        LSUM += Gpredict(PC, spec_.L_shist[VTAGE_INDLOCAL], Lm, LGEHL, LNB, LOGLNB, WL);
        LSUM += Gpredict(PC, spec_.S_slhist[VTAGE_INDSLOCAL], Sm, SGEHL, SNB, LOGSNB, WS);
        LSUM += Gpredict(PC, spec_.T_slhist[VTAGE_INDTLOCAL], Tm, TGEHL, TNB, LOGTNB, WT);
        LSUM += Gpredict(PC, spec_.IMHIST[(spec_.IMLIcount)], IMm, IMGEHL, IMNB, LOGIMNB, WIM);
        LSUM += Gpredict(PC, spec_.IMLIcount, Im, IGEHL, INB, LOGINB, WI);

        C.LSUM = LSUM;

        const bool SCPRED = (LSUM >= 0);

        C.THRES = (updatethreshold >> 3) + Pupdatethreshold[VTAGE_INDUPD] +
                  12 * ((WB[VTAGE_INDUPDS] >= 0) + (WP[VTAGE_INDUPDS] >= 0) + (WS[VTAGE_INDUPDS] >= 0) +
                        (WT[VTAGE_INDUPDS] >= 0) + (WL[VTAGE_INDUPDS] >= 0) + (WG[VTAGE_INDUPDS] >= 0) +
                        (WI[VTAGE_INDUPDS] >= 0));

        const int THRES = C.THRES;

        if ( C.pred_inter != SCPRED ) {
            C.pred_taken = SCPRED;
            if ( C.HighConf ) {
                if ( (abs(LSUM) < THRES / 4) ) { C.pred_taken = C.pred_inter; }
                else if ( (abs(LSUM) < THRES / 2) )
                    C.pred_taken = (SecondH < 0) ? SCPRED : C.pred_inter;
            }

            if ( C.MedConf )
                if ( (abs(LSUM) < THRES / 4) ) { C.pred_taken = (FirstH < 0) ? SCPRED : C.pred_inter; }
        }

        return C.pred_taken;
    }

    // ---- history advance ---------------------------------------------------
    void HistoryUpdate(History& H, const uint64_t PC, const VanadisBranchClass cls, const bool taken,
                       const uint64_t target)
    {
        int brtype = 0;

        switch ( cls ) {
        case VanadisBranchClass::INDIRECT_JUMP:
        case VanadisBranchClass::INDIRECT_CALL:
        case VanadisBranchClass::RETURN:
            brtype = 2;
            break;
        case VanadisBranchClass::DIRECT_JUMP:
        case VanadisBranchClass::DIRECT_CALL:
            brtype = 0;
            break;
        case VanadisBranchClass::CONDITIONAL:
            brtype = 1;
            break;
        }

        int maxt = 2;
        if ( brtype & 1 ) maxt = 2;
        else if ( (brtype & 2) )
            maxt = 3;

        if ( brtype & 1 ) {
            H.IMHIST[H.IMLIcount] = (H.IMHIST[H.IMLIcount] << 1) + taken;

            if ( target < PC ) {
                if ( !taken ) { H.IMLIcount = 0; }
                if ( taken ) {
                    if ( H.IMLIcount < ((1 << Im[0]) - 1) ) H.IMLIcount++;
                }
            }
        }

        if ( brtype & 1 ) {
            const int INDLOCAL  = (int)((PC ^ (PC >> 2)) & (NLOCAL - 1));
            const int INDSLOCAL = (int)(((PC ^ (PC >> 5))) & (NSECLOCAL - 1));
            const int INDTLOCAL = (int)(((PC ^ (PC >> (LOGTNB)))) & (NTLOCAL - 1));

            H.GHIST              = (H.GHIST << 1) + (taken & (target < PC));
            H.L_shist[INDLOCAL]  = (H.L_shist[INDLOCAL] << 1) + (taken);
            H.S_slhist[INDSLOCAL] = ((H.S_slhist[INDSLOCAL] << 1) + taken) ^ (PC & 15);
            H.T_slhist[INDTLOCAL] = (H.T_slhist[INDTLOCAL] << 1) + taken;
        }

        int T    = ((PC ^ (PC >> 2))) ^ taken;
        int PATH = PC ^ (PC >> 2) ^ (PC >> 4);
        if ( (brtype == 3) & taken ) {
            T    = (T ^ (target >> 2));
            PATH = PATH ^ (target >> 2) ^ (target >> 4);
        }

        for ( int t = 0; t < maxt; t++ ) {
            bool DIR     = (T & 1);
            T >>= 1;
            int PATHBIT = (PATH & 127);
            PATH >>= 1;

            H.ptghist--;
            H.ghist[H.ptghist & (HISTBUFFERLENGTH - 1)] = DIR;
            H.phist                                     = (H.phist << 1) ^ PATHBIT;

            for ( int i = 1; i <= NHIST; i++ ) {
                H.ch_i[i].update(H.ghist, H.ptghist);
                H.ch_t[0][i].update(H.ghist, H.ptghist);
                H.ch_t[1][i].update(H.ghist, H.ptghist);
            }
        }

        H.phist = (H.phist & ((1 << PHISTWIDTH) - 1));
    }

    // ---- training ----------------------------------------------------------
    void UpdatePredictor(const uint64_t PC, const bool resolveDir, Checkpoint& C)
    {
        // Everything the index macros read comes from the record, so an index
        // computed here is the index the prediction used.
        const int*      GI               = C.GI;
        const uint32_t* GTAG             = C.GTAG;
        const bool      LowConf          = C.LowConf;
        const bool      HighConf         = C.HighConf;
        const bool      MedConf          = C.MedConf;
        const bool      AltConf          = C.AltConf;
        const bool      LongestMatchPred = C.LongestMatchPred;
        const bool      alttaken         = C.alttaken;
        const bool      pred_inter       = C.pred_inter;
        const int       HitBank          = C.HitBank;
        const int       AltBank          = C.AltBank;
        const int       LSUM             = C.LSUM;
        const int       THRES            = C.THRES;

        if ( C.LVALID ) {
            if ( C.pred_taken != C.predloop ) ctrupdate(WITHLOOP, (C.predloop == resolveDir), 7);
        }
        loopupdate(resolveDir, (C.pred_taken != resolveDir), C);

        const bool SCPRED = (LSUM >= 0);
        if ( pred_inter != SCPRED ) {
            if ( (abs(LSUM) < THRES) )
                if ( (HighConf) ) {
                    if ( (abs(LSUM) < THRES / 2) )
                        if ( (abs(LSUM) >= THRES / 4) ) ctrupdate(SecondH, (pred_inter == resolveDir), CONFWIDTH);
                }
            if ( (MedConf) )
                if ( (abs(LSUM) < THRES / 4) ) { ctrupdate(FirstH, (pred_inter == resolveDir), CONFWIDTH); }
        }

        if ( (SCPRED != resolveDir) || ((abs(LSUM) < THRES)) ) {
            {
                if ( SCPRED != resolveDir ) {
                    Pupdatethreshold[VTAGE_INDUPD] += 1;
                    updatethreshold += 1;
                }
                else {
                    Pupdatethreshold[VTAGE_INDUPD] -= 1;
                    updatethreshold -= 1;
                }

                if ( Pupdatethreshold[VTAGE_INDUPD] >= (1 << (WIDTHRESP - 1)) )
                    Pupdatethreshold[VTAGE_INDUPD] = (1 << (WIDTHRESP - 1)) - 1;
                if ( Pupdatethreshold[VTAGE_INDUPD] < -(1 << (WIDTHRESP - 1)) )
                    Pupdatethreshold[VTAGE_INDUPD] = -(1 << (WIDTHRESP - 1));
                if ( updatethreshold >= (1 << (WIDTHRES - 1)) ) updatethreshold = (1 << (WIDTHRES - 1)) - 1;
                if ( updatethreshold < -(1 << (WIDTHRES - 1)) ) updatethreshold = -(1 << (WIDTHRES - 1));
            }
            {
                int XSUM = LSUM - ((WB[VTAGE_INDUPDS] >= 0) * ((2 * Bias[VTAGE_INDBIAS] + 1) +
                                                               (2 * BiasSK[VTAGE_INDBIASSK] + 1) +
                                                               (2 * BiasBank[VTAGE_INDBIASBANK] + 1)));
                if ( (XSUM + ((2 * Bias[VTAGE_INDBIAS] + 1) + (2 * BiasSK[VTAGE_INDBIASSK] + 1) +
                              (2 * BiasBank[VTAGE_INDBIASBANK] + 1)) >=
                      0) != (XSUM >= 0) )
                    ctrupdate(WB[VTAGE_INDUPDS], (((2 * Bias[VTAGE_INDBIAS] + 1) + (2 * BiasSK[VTAGE_INDBIASSK] + 1) +
                                                   (2 * BiasBank[VTAGE_INDBIASBANK] + 1) >=
                                                   0) == resolveDir),
                              EWIDTH);
            }
            ctrupdate(Bias[VTAGE_INDBIAS], resolveDir, PERCWIDTH);
            ctrupdate(BiasSK[VTAGE_INDBIASSK], resolveDir, PERCWIDTH);
            ctrupdate(BiasBank[VTAGE_INDBIASBANK], resolveDir, PERCWIDTH);

            // The history values these are indexed by are the ones the
            // prediction saw, taken from the record: the speculative copy has
            // moved on and the architected copy has not necessarily reached
            // the same place by a different route.
            Gupdate((PC << 1) + pred_inter, resolveDir, C.s_GHIST, Gm, GGEHL, GNB, LOGGNB, WG, LSUM);
            Gupdate(PC, resolveDir, C.s_phist, Pm, PGEHL, PNB, LOGPNB, WP, LSUM);
            Gupdate(PC, resolveDir, C.s_L_shist, Lm, LGEHL, LNB, LOGLNB, WL, LSUM);
            Gupdate(PC, resolveDir, C.s_S_slhist, Sm, SGEHL, SNB, LOGSNB, WS, LSUM);
            Gupdate(PC, resolveDir, C.s_T_slhist, Tm, TGEHL, TNB, LOGTNB, WT, LSUM);
            Gupdate(PC, resolveDir, C.s_IMHIST, IMm, IMGEHL, IMNB, LOGIMNB, WIM, LSUM);
            Gupdate(PC, resolveDir, C.s_IMLIcount, Im, IGEHL, INB, LOGINB, WI, LSUM);
        }

        // TAGE
        bool ALLOC = ((C.tage_pred != resolveDir) & (HitBank < NHIST));

        if ( HitBank > 0 ) {
            bool PseudoNewAlloc = (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) <= 1);
            if ( PseudoNewAlloc ) {
                if ( LongestMatchPred == resolveDir ) ALLOC = false;

                if ( LongestMatchPred != alttaken ) {
                    ctrupdate(use_alt_on_na[VTAGE_INDUSEALT], (alttaken == resolveDir), ALTWIDTH);
                }
            }
        }

        if ( C.pred_taken == resolveDir )
            if ( (MYRANDOM() & 31) != 0 ) ALLOC = false;

        if ( ALLOC ) {
            int T = NNN;

            int A = 1;
            if ( (MYRANDOM() & 127) < 32 ) A = 2;
            int Penalty = 0;
            int NA      = 0;
            int DEP     = ((((HitBank - 1 + 2 * A) & 0xffe)) ^ (MYRANDOM() & 1));

            for ( int I = DEP; I < NHIST; I += 2 ) {
                int  i    = I + 1;
                bool Done = false;
                if ( NOSKIP[i] ) {
                    if ( gtable[i][GI[i]].u == 0 ) {
                        if ( abs(2 * gtable[i][GI[i]].ctr + 1) <= 3 ) {
                            gtable[i][GI[i]].tag = GTAG[i];
                            gtable[i][GI[i]].ctr = (resolveDir) ? 0 : -1;
                            NA++;
                            if ( T <= 0 ) { break; }
                            I += 2;
                            Done = true;
                            T -= 1;
                        }
                        else {
                            if ( gtable[i][GI[i]].ctr > 0 ) gtable[i][GI[i]].ctr--;
                            else
                                gtable[i][GI[i]].ctr++;
                        }
                    }
                    else {
                        Penalty++;
                    }
                }

                if ( !Done ) {
                    i = (I ^ 1) + 1;
                    if ( NOSKIP[i] ) {
                        if ( gtable[i][GI[i]].u == 0 ) {
                            if ( abs(2 * gtable[i][GI[i]].ctr + 1) <= 3 ) {
                                gtable[i][GI[i]].tag = GTAG[i];
                                gtable[i][GI[i]].ctr = (resolveDir) ? 0 : -1;
                                NA++;
                                if ( T <= 0 ) { break; }
                                I += 2;
                                T -= 1;
                            }
                            else {
                                if ( gtable[i][GI[i]].ctr > 0 ) gtable[i][GI[i]].ctr--;
                                else
                                    gtable[i][GI[i]].ctr++;
                            }
                        }
                        else {
                            Penalty++;
                        }
                    }
                }
            }
            TICK += (Penalty - 2 * NA);

            if ( TICK < 0 ) TICK = 0;
            if ( TICK >= BORNTICK ) {
                for ( int i = 1; i <= BORN; i += BORN - 1 )
                    for ( int j = 0; j < SizeTable[i]; j++ )
                        gtable[i][j].u >>= 1;
                TICK = 0;
            }
        }

        if ( HitBank > 0 ) {
            if ( abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) == 1 )
                if ( LongestMatchPred != resolveDir ) {
                    if ( AltBank > 0 ) { ctrupdate(gtable[AltBank][GI[AltBank]].ctr, resolveDir, CWIDTH); }
                    if ( AltBank == 0 ) baseupdate(resolveDir, C);
                }
            ctrupdate(gtable[HitBank][GI[HitBank]].ctr, resolveDir, CWIDTH);
            if ( abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) == 1 ) gtable[HitBank][GI[HitBank]].u = 0;
            if ( alttaken == resolveDir )
                if ( AltBank > 0 )
                    if ( abs(2 * gtable[AltBank][GI[AltBank]].ctr + 1) == 7 )
                        if ( gtable[HitBank][GI[HitBank]].u == 1 ) {
                            if ( LongestMatchPred == resolveDir ) { gtable[HitBank][GI[HitBank]].u = 0; }
                        }
        }
        else
            baseupdate(resolveDir, C);

        if ( LongestMatchPred != alttaken )
            if ( LongestMatchPred == resolveDir ) {
                if ( gtable[HitBank][GI[HitBank]].u < (1 << UWIDTH) - 1 ) gtable[HitBank][GI[HitBank]].u++;
            }

        HistoryUpdate(arch_, PC, C.cls, resolveDir, C.hist_target);
    }

    // ---- the loop predictor ------------------------------------------------
    static int lindex(uint64_t PC) { return (((PC ^ (PC >> 2)) & ((1 << (LOGL - 2)) - 1)) << 2); }

    bool getloop(uint64_t PC, Checkpoint& C)
    {
        C.LHIT = -1;

        C.LI   = lindex(PC);
        C.LIB  = ((PC >> (LOGL - 2)) & ((1 << (LOGL - 2)) - 1));
        C.LTAG = (PC >> (LOGL - 2)) & ((1 << 2 * LOOPTAG) - 1);
        C.LTAG ^= (C.LTAG >> LOOPTAG);
        C.LTAG = (C.LTAG & ((1 << LOOPTAG) - 1));

        for ( int i = 0; i < 4; i++ ) {
            int index = (C.LI ^ ((C.LIB >> i) << 2)) + i;

            if ( ltable[index].TAG == C.LTAG ) {
                C.LHIT   = i;
                C.LVALID = ((ltable[index].confid == CONFLOOP) || (ltable[index].confid * ltable[index].NbIter > 128));

                if ( ltable[index].CurrentIter + 1 == ltable[index].NbIter ) return (!(ltable[index].dir));
                return ((ltable[index].dir));
            }
        }

        C.LVALID = false;
        return (false);
    }

    void loopupdate(bool Taken, bool ALLOC, const Checkpoint& C)
    {
        if ( C.LHIT >= 0 ) {
            int index = (C.LI ^ ((C.LIB >> C.LHIT) << 2)) + C.LHIT;
            if ( C.LVALID ) {
                if ( Taken != C.predloop ) {
                    ltable[index].NbIter      = 0;
                    ltable[index].age         = 0;
                    ltable[index].confid      = 0;
                    ltable[index].CurrentIter = 0;
                    return;
                }
                else if ( (C.predloop != C.tage_pred) || ((MYRANDOM() & 7) == 0) )
                    if ( ltable[index].age < CONFLOOP ) ltable[index].age++;
            }

            ltable[index].CurrentIter++;
            ltable[index].CurrentIter &= ((1 << WIDTHNBITERLOOP) - 1);
            if ( ltable[index].CurrentIter > ltable[index].NbIter ) {
                ltable[index].confid = 0;
                ltable[index].NbIter = 0;
            }
            if ( Taken != ltable[index].dir ) {
                if ( ltable[index].CurrentIter == ltable[index].NbIter ) {
                    if ( ltable[index].confid < CONFLOOP ) ltable[index].confid++;
                    if ( ltable[index].NbIter < 3 ) {
                        ltable[index].dir    = Taken;
                        ltable[index].NbIter = 0;
                        ltable[index].age    = 0;
                        ltable[index].confid = 0;
                    }
                }
                else {
                    if ( ltable[index].NbIter == 0 ) {
                        ltable[index].confid = 0;
                        ltable[index].NbIter = ltable[index].CurrentIter;
                    }
                    else {
                        ltable[index].NbIter = 0;
                        ltable[index].confid = 0;
                    }
                }
                ltable[index].CurrentIter = 0;
            }
        }
        else if ( ALLOC ) {
            uint64_t X = MYRANDOM() & 3;

            if ( (MYRANDOM() & 3) == 0 )
                for ( int i = 0; i < 4; i++ ) {
                    int LHIT_aux = (X + i) & 3;
                    int index    = (C.LI ^ ((C.LIB >> LHIT_aux) << 2)) + LHIT_aux;
                    if ( ltable[index].age == 0 ) {
                        ltable[index].dir         = !Taken;
                        ltable[index].TAG         = C.LTAG;
                        ltable[index].NbIter      = 0;
                        ltable[index].age         = 7;
                        ltable[index].confid      = 0;
                        ltable[index].CurrentIter = 0;
                        break;
                    }
                    else
                        ltable[index].age--;
                    break;
                }
        }
    }

    // ---- the tables, none of which is speculative --------------------------
    int8_t Bias[(1 << LOGBIAS)]     = {0};
    int8_t BiasSK[(1 << LOGBIAS)]   = {0};
    int8_t BiasBank[(1 << LOGBIAS)] = {0};

    int    Im[INB]                     = {8};
    int8_t IGEHLA[INB][(1 << LOGINB)]  = {{0}};
    int8_t* IGEHL[INB]                 = {nullptr};

    int    IMm[IMNB]                     = {10, 4};
    int8_t IMGEHLA[IMNB][(1 << LOGIMNB)] = {{0}};
    int8_t* IMGEHL[IMNB]                 = {nullptr};

    int    Gm[GNB]                    = {40, 24, 10};
    int8_t GGEHLA[GNB][(1 << LOGGNB)] = {{0}};
    int8_t* GGEHL[GNB]                = {nullptr};

    int    Pm[PNB]                    = {25, 16, 9};
    int8_t PGEHLA[PNB][(1 << LOGPNB)] = {{0}};
    int8_t* PGEHL[PNB]                = {nullptr};

    int    Lm[LNB]                    = {11, 6, 3};
    int8_t LGEHLA[LNB][(1 << LOGLNB)] = {{0}};
    int8_t* LGEHL[LNB]                = {nullptr};

    int    Sm[SNB]                    = {16, 11, 6};
    int8_t SGEHLA[SNB][(1 << LOGSNB)] = {{0}};
    int8_t* SGEHL[SNB]                = {nullptr};

    int    Tm[TNB]                    = {9, 4};
    int8_t TGEHLA[TNB][(1 << LOGTNB)] = {{0}};
    int8_t* TGEHL[TNB]                = {nullptr};

    int    updatethreshold                 = 0;
    int    Pupdatethreshold[(1 << LOGSIZEUP)] = {0};
    int8_t WG[(1 << LOGSIZEUPS)]  = {0};
    int8_t WL[(1 << LOGSIZEUPS)]  = {0};
    int8_t WS[(1 << LOGSIZEUPS)]  = {0};
    int8_t WT[(1 << LOGSIZEUPS)]  = {0};
    int8_t WP[(1 << LOGSIZEUPS)]  = {0};
    int8_t WI[(1 << LOGSIZEUPS)]  = {0};
    int8_t WIM[(1 << LOGSIZEUPS)] = {0};
    int8_t WB[(1 << LOGSIZEUPS)]  = {0};

    int8_t FirstH  = 0;
    int8_t SecondH = 0;
    int8_t use_alt_on_na[SIZEUSEALT] = {0};

    bentry* btable = nullptr;
    gentry* gtable[NHIST + 1] = {nullptr};
    lentry* ltable = nullptr;

    int8_t WITHLOOP = -1;
    int    TICK     = 0;
    int    Seed     = 0;

    int  m[NHIST + 1]         = {0};
    int  TB[NHIST + 1]        = {0};
    int  logg[NHIST + 1]      = {0};
    bool NOSKIP[NHIST + 1]    = {false};
    int  SizeTable[NHIST + 1] = {0};

    History spec_;
    History arch_;
};

#undef VTAGE_INDBIAS
#undef VTAGE_INDBIASSK
#undef VTAGE_INDBIASBANK
#undef VTAGE_INDLOCAL
#undef VTAGE_INDSLOCAL
#undef VTAGE_INDTLOCAL
#undef VTAGE_INDUPD
#undef VTAGE_INDUPDS
#undef VTAGE_INDUSEALT
#undef VTAGE_GINDEX

} // namespace Vanadis
} // namespace SST

#endif
