INC_DIRS := ./
SRC_DIRS := ./
OBJ_DIR := ./obj
BIN_DIR := .
EXE_NAME := homebot.out

INC := $(foreach d, $(INC_DIRS), -I$d)
SRC := $(foreach d, $(SRC_DIRS), $(wildcard $d/*.cpp))
OBJ := $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SRC)))
EXE := $(BIN_DIR)/$(EXE_NAME)

CXX	:= g++
CXXFLAGS := -std=c++17 -Wall $(INC) -MMD -MP -Wfatal-errors -O3 -fopenmp
LDFLAGS := -fopenmp

ifeq ($(shell grep -q avx512f /proc/cpuinfo && echo -n yes),yes)
    CXXFLAGS += -mavx512f
endif
ifeq ($(shell grep -q avx512vl /proc/cpuinfo && echo -n yes),yes)
    CXXFLAGS += -mavx512vl
endif
ifeq ($(shell grep -q avx512bw /proc/cpuinfo && echo -n yes),yes)
    CXXFLAGS += -mavx512bw
endif
ifeq ($(shell grep -q avx512dq /proc/cpuinfo && echo -n yes),yes)
    CXXFLAGS += -mavx512dq
endif

.PHONY: all clean deepclean debug performance

all: $(EXE)

performance: CXXFLAGS += -DNASSERT
performance: $(EXE)

debug: CXXFLAGS += -DDEBUG -g
debug: $(EXE)

datarace:
	$(CXX) $(SRC) $(INC) $(LDFLAGS) -fsanitize=thread -fPIE -pie -o $(EXE)

$(EXE): $(OBJ) | $(BIN_DIR)
	$(CXX) $^ $(LDFLAGS) -o $@

$(BIN_DIR) $(OBJ_DIR):
	mkdir -p $@

$(OBJ) : %.o: $(SRC) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $(filter %$(notdir $*).cpp,$^) -o $@

deepclean: clean
	@$(RM) -rv *.out *.o || true

clean:
	@$(RM) -rv $(EXE) $(BIN_DIR) $(OBJ_DIR) build.log || true

-include $(OBJ:.o=.d)
