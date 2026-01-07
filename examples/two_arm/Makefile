# compiler
CXX       = clang++
CXXFLAGS  = -Wall -Wextra -std=c++11 -O3 -DNDEBUG -stdlib=libc++
LDFLAGS   = -stdlib=libc++

# scots
SCOTSROOT = ../..
SCOTSINC  = -I$(SCOTSROOT)/src -I$(SCOTSROOT)/utils

.PHONY: all clean two_arm_scots simulate
TARGET = two_arm_scots simulate

all: $(TARGET)

two_arm_scots:
	$(CXX) $(CXXFLAGS) $(SCOTSINC) two_arm_scots.cc -o two_arm_scots $(LDFLAGS)

clean:
	rm -f two_arm_scots

