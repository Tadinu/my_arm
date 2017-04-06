##
##
##
##	MAKEFILE FOR BUILDING XCS 
##
##
##

CXX = g++
OPT = -g
EXTCXXFLAGS := -I. $(CXXFLAGS) -static $(EXTERNAL_INC) $(EXTERNAL_LIB)

### suffix for the executable
VERSION = mp

### class to define environmental state
INPUTS = binary_inputs

### environment definition
ENVIRONMENT = multiplexer_env

### classifier definitions
ACTIONS = boolean_action
CONDITIONS = ternary_condition
CLASSIFIER = xcs
MODEL = xcs

### class which runs the experiments
EXPERIMENT_MANAGER = experiment_mgr
OTHER_OBJ = generic.o $(EXTERNAL_OBJ)
.PHONY: CLEAN 

# classes for various utilities
UTILITY = xcs_utility.o xcs_random.o xcs_config_mgr2.o 

### core classes
CORESYS = \
	$(CLASSIFIER)_main.o $(CLASSIFIER)_classifier.o \
	$(CLASSIFIER)_classifier_system.o $(EXPERIMENT_MANAGER).o $(UTILITY)

# files oggetto classi esterne per dipendenze 
EXTSYS = $(INPUTS).o $(CONDITIONS).o $(ACTIONS).o

# header files classi esterne da includere durante la compilazione
EXTSYS_INCLUDE = \
	-D __DET_INCLUDE__='"$(INPUTS).hpp"' \
	-D __COND_INCLUDE__='"$(CONDITIONS).hpp"' \
	-D __ACT_INCLUDE__='"$(ACTIONS).hpp"' \
	-D __CLS_INCLUDE__='"$(CLASSIFIER)_classifier.hpp"' \
	-D __MOD_INCLUDE__='"$(CLASSIFIER)_classifier_system.hpp"' 
	
ENV_INCLUDE = \
	-D __ENV_INCLUDE__='"$(ENVIRONMENT).hpp"' 
	
### the name of the external classes for actions, conditions, sensors, and environment
### are added to the source files as #define statements
ENV_CLASSDEF = \
	-D __ENVIRONMENT__=$(ENVIRONMENT)
	
EXTSYS_CLASSDEF = \
	-D __INPUTS__=$(INPUTS) \
	-D __ACTION__=$(ACTIONS) \
	-D __CONDITION__=$(CONDITIONS) \
	-D __CLASSIFIER__=$(CLASSIFIER)_classifier \
	-D __MODEL__=$(MODEL)_classifier_system
	
### compile options
override CXXFLAGS := $(EXTCXXFLAGS) $(OPT) $(EXTSYS_CLASSDEF) $(ENV_CLASSDEF) $(EXTSYS_INCLUDE) $(ENV_INCLUDE) 

### build the overall system
XCS: $(CLASSIFIER)_main.o $(CORESYS) $(EXTSYS) $(ENVIRONMENT).o $(OTHER_OBJ)
	$(CXX) $(CXXFLAGS) -o $(CLASSIFIER)-$(VERSION) \
	$(CORESYS) $(EXTSYS) $(ENVIRONMENT).o $(OTHER_OBJ) -lm

MATCH: match.o $(CLASSIFIER)_classifier.o $(UTILITY) $(EXTSYS) $(OTHER_OBJ) 
	$(CXX) $(CXXFLAGS) -o match-$(VERSION) match.o $(CLASSIFIER)_classifier.o $(UTILITY) $(EXTSYS) $(OTHER_OBJ) -lz

PRINT: print_inputs.o $(UTILITY) $(ENVIRONMENT).o $(INPUTS).o $(OTHER_OBJ)
	$(CXX) $(CXXFLAGS) $(ENV_INCLUDE) -o print-$(VERSION) print_inputs.o $(UTILITY) $(ENVIRONMENT).o $(INPUTS).o $(OTHER_OBJ)
