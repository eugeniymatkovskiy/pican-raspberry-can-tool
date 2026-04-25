CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
LDFLAGS  ?=

BIN_DIR   := build

OBD_SRCS  := advanced_smart_reader.cpp obd2_parser.cpp
OBD_BIN   := $(BIN_DIR)/obd2_reader

UDS_SRCS  := uds/main.cpp uds/iso_tp.cpp uds/uds_client.cpp uds/vw_mqb.cpp uds/backup.cpp
UDS_BIN   := $(BIN_DIR)/uds_main

TEST_LIB_SRCS := uds/iso_tp.cpp uds/uds_client.cpp uds/vw_mqb.cpp uds/backup.cpp
TEST_SRCS     := tests/isotp_loopback.cpp $(TEST_LIB_SRCS)
TEST_BIN      := $(BIN_DIR)/isotp_loopback

.PHONY: all clean obd2 uds tests test

all: obd2 uds

obd2: $(OBD_BIN)
uds:  $(UDS_BIN)
tests: $(TEST_BIN)

$(OBD_BIN): $(OBD_SRCS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OBD_SRCS) -o $@ $(LDFLAGS)

$(UDS_BIN): $(UDS_SRCS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -Iuds $(UDS_SRCS) -o $@ $(LDFLAGS)

$(TEST_BIN): $(TEST_SRCS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -Iuds $(TEST_SRCS) -o $@ $(LDFLAGS) -pthread

test: $(TEST_BIN)
	$(TEST_BIN)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

clean:
	rm -rf $(BIN_DIR)
