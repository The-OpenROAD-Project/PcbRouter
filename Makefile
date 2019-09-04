CC=gcc
CXX=g++
RM=rm -f
CPPFLAGS=-g -std=c++14 -Wall $(shell root-config --cflags) 
LDFLAGS=-g  $(shell root-config --ldflags)
LDLIBS=$(shell root-config --libs)

SRCS=main.cpp kicadPcbDataBase.cpp pathfinder.cpp BoardGrid.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

all: PcbRouter

PcbRouter: $(OBJS)
	$(CXX) $(LDFLAGS) -o PcbRouter $(OBJS) $(LDLIBS) 

BoardGrid.o: BoardGrid.cpp BoardGrid.h

clean:
	$(RM) $(OBJS)
	$(RM) PcbRouter

distclean: clean
	$(RM) PcbRouter

test: PcbRouter
	./PcbRouter
