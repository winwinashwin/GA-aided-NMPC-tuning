SHELL          := /bin/bash
.DEFAULT_GOAL  := all

CC   = g++
PY   = python

DEBUG   ?= 0

SRC_DIR    = ./src
OBJ_DIR    = ./obj
INC_DIR    = ./third_party/Eigen-3.3 /usr/include/python3.8

SRCS       = $(wildcard ${SRC_DIR}/*.cpp)
OBJS       = $(addsuffix .o, $(basename $(addprefix ${OBJ_DIR}/, $(notdir ${SRCS}))))
TESTS      = $(wildcard tests/test_*.py)
BMS        = $(wildcard benchmark/bm_*.py)

C_FLAGS    = -fPIC

ifeq (${DEBUG}, 1)
    C_FLAGS += -g3 -gdwarf
else
    C_FLAGS += -O3 -DNDEBUG
endif

LIBS       = boost_python38 ipopt

INC_FLAGS  = $(addprefix -I, ${INC_DIR})
LD_FLAGS   = $(addprefix -l, ${LIBS})

TARGET     = nmpc.so

.PHONY: all init test bench clean format
.SILENT: all init test bench format

all: init ${TARGET} test bench
	printf '=%.0s' {1..70}; echo

init:
	mkdir -p ${OBJ_DIR}
	rm -f ${TARGET}

${OBJ_DIR}/%.o: ${SRC_DIR}/%.cpp
	$(CC) -c ${C_FLAGS} ${INC_FLAGS} -o $@ $<

${TARGET}: ${OBJS}
	$(CC) -shared $^ ${LD_FLAGS} -o ${TARGET}

test: ${TESTS}
	printf '=%.0s' {1..70}; echo
	$(PY) -m unittest ${TESTS}

bench: ${BMS}
	printf '=%.0s' {1..70}; echo
	for bm in $^; do $(PY) $$bm; done

clean:
	rm -rf ${OBJ_DIR} ${TARGET}

format:
	flake8 .