# The branch predictor's standalone tests.
#
# Three programs, built with the element's own compiler and flags but with none
# of the simulator linked in, all driven by a branch trace recorded from a
# workload run:
#
#   refdrive   the UNMODIFIED reference TAGE-SC-L, given each branch's outcome
#              immediately after its prediction, which is the only way it can
#              be driven and the conditions the port has to reproduce
#   refdrive_hp  the same for the unmodified reference hashed perceptron
#   portdrive  the ported predictors, with the speculative history live, with
#              wrong paths injected, and with a repair after each one
#
# To record a trace of your own:
#
#   NMFC_BRANCH_TRACE=<file> NMFC_BRANCH_TRACE_RECORDS=200000 \
#     sst .../vanadis-nmfc.py
#
# The trace committed here is 131,501 retired branches of the BFS workload on
# this core, recorded the way described above and stored compressed.
#
# make            build all three and run them against host_bfs.btrace
# make TRACE=...  the same against another trace

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -g -Wall

VANADIS  := ../..
REFDIR   := /mnt/md0/ChampSim/ChampSimDPC4/branch/tage_sc_l
HPDIR    := /mnt/md0/ChampSim/base_wt/branch/hashed_perceptron
HPINC    := /mnt/md0/ChampSim/base_wt/inc
FMTINC   := /mnt/md0/NMFC-Rev/src/ramulator2/ext/fmt/include

TRACE    ?= host_bfs.btrace
WORK     ?= ./out
LIMIT    ?= 0
STAGE2   ?= 4000

.PHONY: all check clean

all: check

refdrive: refdrive.cc branchtrace.h
	$(CXX) $(CXXFLAGS) -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare \
	  -I. -Ichampsim_shim -I$(REFDIR) -o $@ $<

refdrive_hp: refdrive_hp.cc branchtrace.h
	$(CXX) $(CXXFLAGS) -fno-access-control -Wno-unused-variable -Wno-sign-compare \
	  -I. -Ichampsim_shim_hp -I$(HPINC) -I$(HPDIR) -I$(FMTINC) -o $@ $<

portdrive: portdrive.cc branchtrace.h $(VANADIS)/vbranch/vbranchtagescl.h \
           $(VANADIS)/vbranch/vbranchperceptron.h $(VANADIS)/vbranch/vbranchspeccore.h \
           $(VANADIS)/vbranch/vbranchcheckpoint.h
	$(CXX) $(CXXFLAGS) -Wno-sign-compare -I. -I$(VANADIS) -o $@ $<

%.btrace: %.btrace.gz
	gunzip -c $< > $@

check: refdrive refdrive_hp portdrive $(TRACE)
	@mkdir -p $(WORK)
	./refdrive $(TRACE) $(WORK)/ref $(LIMIT)
	./refdrive_hp $(TRACE) $(WORK)/refhp $(LIMIT)
	./portdrive $(TRACE) $(WORK)/ref $(WORK)/refhp $(LIMIT) $(STAGE2)

clean:
	rm -f refdrive refdrive_hp portdrive host_bfs.btrace
	rm -rf $(WORK)
