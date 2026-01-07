#
# matlab root 
#
MATLABROOT	= /usr/local/MATLAB/R2024b/
#
# compiler
#
CC       	  = $(MATLABROOT)/bin/mex
CXXFLAGS 		= -g -Wall -Wextra -std=c++11 -O3 
#
# scots
#
SCOTSROOT   = ../..
SCOTSINC		= -I$(SCOTSROOT)/src
#
# cudd 
#
CUDDPATH		=  ../../cudd-3.0.0/
CUDDINC 		= -I$(CUDDPATH)/include -I$(CUDDPATH)
CUDDLIBS		= -lcudd 
CUDDLPATH   = -L$(CUDDPATH)/lib



all: sparse bdd

sparse: GridPoints mexStaticController

bdd: mexSymbolicSet


mexStaticController: mexStaticController.cc
	$(CC) COMPFLAGS='$(COMPFLAGS) $(CXXFLAGS)' -largeArrayDims $(SCOTSINC) mexStaticController.cc

GridPoints: GridPoints.cc
	$(CC) COMPFLAGS='$(COMPFLAGS) $(CXXFLAGS)' -largeArrayDims $(SCOTSINC) GridPoints.cc

mexSymbolicSet: mexSymbolicSet.cc
	$(CC) COMPFLAGS='$(COMPFLAGS) $(CXXFLAGS)' -DSCOTS_BDD -largeArrayDims $(SCOTSINC) $(CUDDINC) $(CUDDLPATH) $(CUDDLIBS) mexSymbolicSet.cc

