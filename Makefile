CXX ?= c++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra
CPPFLAGS ?= -Iinclude
LDFLAGS ?=

.PHONY: all test clean

all: chip_tests chip_render chip_demo

chip_tests: src/chip.cpp tests/chip_tests.cpp include/ssm2045/chip.h
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/chip.cpp tests/chip_tests.cpp -o chip_tests $(LDFLAGS)

chip_render: src/chip.cpp examples/render.cpp include/ssm2045/chip.h
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/chip.cpp examples/render.cpp -o chip_render $(LDFLAGS)

chip_demo: src/chip.cpp examples/chip_demo.cpp include/ssm2045/chip.h
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/chip.cpp examples/chip_demo.cpp -o chip_demo $(LDFLAGS)

test: chip_tests
	./chip_tests

clean:
	rm -f chip_tests chip_render chip_demo
