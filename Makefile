CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Wpedantic
INCLUDES ?= -I/usr/include/eigen3
BUILD    := build
TARGET   := $(BUILD)/ekf_6dof.o

.PHONY: all clean calib

all: $(TARGET)

$(BUILD):
	@mkdir -p $(BUILD)

$(TARGET): src/ekf_6dof.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

calib:
	python3 scripts/allan_variance_calib.py data/mpu6050_bench_raw.csv

clean:
	@rm -rf $(BUILD)
