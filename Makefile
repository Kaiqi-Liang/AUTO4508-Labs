# Makefile T. Braunl 2017, Compile all .cpp into .x
ALL:  $(patsubst %.cpp,%.x, $(wildcard */*.cpp))

%.x: %.cpp
	g++sim -o $*.x $*.cpp

clean:
	$(RM) */*.x

