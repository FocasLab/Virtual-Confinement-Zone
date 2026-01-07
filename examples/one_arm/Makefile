# compiler
CXX       = clang++
CXXFLAGS  = -Wall -Wextra -std=c++11 -O3 -DNDEBUG -stdlib=libc++
LDFLAGS   = -stdlib=libc++

# scots
SCOTSROOT = ../..
SCOTSINC  = -I$(SCOTSROOT)/src -I$(SCOTSROOT)/utils/

.PHONY: all clean one_arm_scots
TARGET = one_arm_scots

all: $(TARGET)

one_arm_scots:
	$(CXX) $(CXXFLAGS) $(SCOTSINC) one_arm_scots.cc -o one_arm_scots $(LDFLAGS)

clean:
	rm -f one_arm_scots 

