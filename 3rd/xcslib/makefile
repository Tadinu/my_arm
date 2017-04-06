#	MAKEFILE FOR XCS
#
#	generates different versions of XCS
#
#	multiplexer:	standard XCS with multiplexer problems
#
#	maze:		standard XCS with maze problems
#

EXECUTABLES=`find . -maxdepth 1 -type f -perm +a+x`
OBJECTS=$(wildcard ./*.o ./extra/*.o ./pf/*.o)
DOCS=$(wildcard ./docs/html/*.html ./docs/html/*.png)
BAK = $(wildcard ./*~)
EXECUTION=$(wildcard pop_state.* statistics.* *.gz popul.* steps.* report.* AVERAGE.* average.*)

nothing:
	@echo ""
	@echo "make"
	@echo "   print this help"
	@echo ""
	@echo make all
	@echo "  build all the available executables and install them in HOME/bin"
	@echo ""
	@echo make docs
	@echo "  build html documentation in docs/html"
	@echo ""
	@echo make mp
	@echo "  build the version of XCS to learn Boolean multiplexer"
	@echo ""
	@echo make woods
	@echo "  build the version of XCS to learn the Woods environment"
	@echo ""
	@echo make fa
	@echo "  build the version of XCSF to approximate functions"
	@echo ""
	@echo make gw
	@echo "  build the version of XCSF to solve the 2D gridworld"
	@echo ""
	@echo make utility
	@echo "  build and install the utilities"
	@echo ""

#
#	CLEANING
#
clean:
	@\rm -f $(OBJECTS) $(BAK) $(DOCS)

clean-all:
	@\rm -f $(OBJECTS) $(BAK) $(EXECUTABLES) *.gz
	@\rm -f example/*/*.gz statistics.* trace.* steps.* perf.* popul.* report.* system_error.* AVERAGE.* average.*


.PHONY: docs clean clean-all

#
#	CODE DOCUMENTATION
#
docs:
	doxygen make/xcs-docs.ini

INSTALL=$(HOME)/bin

#
#	BOOLEAN MULTIPLEXER
#


all: mp woods gw fa 
	@cd utility; make install; cd ..; mv $(EXECUTABLES) $(INSTALL)

#
#	2D GRIDWORLD
#
gw:
	make clean; make -f make/xcsf.make ENVIRONMENT=grid_env VERSION=gw ACTIONS=integer_action

#
#	FUNCTION APPROXIMATION
#
fa:
	make clean; make -f make/xcsf.make 

#
#	Boolean multiplexer
#
mp:
	make clean
	make -f make/xcs.make VERSION=mp 

woods:
	make clean
	make -f make/xcs.make VERSION=woods ENVIRONMENT=woods_env ACTIONS=binary_action  

woods2:
	make clean
	make -f make/xcs.make VERSION=woods2 ENVIRONMENT=woods2_env ACTIONS=binary_action  

others:
	make clean
	make -f make/xcs.make VERSION=woods ENVIRONMENT=woods_env ACTIONS=binary_action MATCH
	make clean
	make -f make/xcs.make VERSION=woods ENVIRONMENT=woods_env ACTIONS=binary_action PRINT


