################################################################
#
# flight.c unit tests (based on gcc)
#
################################################################


################################################################
# target
################################################################
TARGET = flight_int

# The corresponding SDEC scripts to execute
TEST_SUITE = Flight
TEST_SUITE_DIR = Testing/Automated/$(TEST_SUITE)
TEST_SUITE_MOD = Testing.Automated.$(TEST_SUITE)

################################################################
# build variables 
################################################################

# Debug build?
DEBUG = 1
# Optimization
OPT   = -Og


################################################################
# paths 
################################################################

# build folder
BUILD_DIR = build

# base directory
# depending on your test, this might not be correct. update the 
# relative path based on where the test is compared to
# Flight-Computer-Firmware.
ROOT_DIR  = ../../../..

# directory shortcuts for includes and sources
LIB_DIR = $(ROOT_DIR)/lib
MOD_DIR = $(ROOT_DIR)/mod
INIT_DIR = $(ROOT_DIR)/init
DRIVER_DIR = $(ROOT_DIR)/driver

################################################################
# source 
################################################################

# NO test sources! All self contained in the framework, we just build from here!

# NO COV SOURCES -- emulator based tests should build the emulator with coverage

# Framework (just these files. do not modify.)
FRAMEWORK_SOURCES =												  \
$(ROOT_DIR)/test/framework/src/test_assert.c          	  		  \
$(ROOT_DIR)/test/framework/src/test_runner.c

C_SOURCES = $(FRAMEWORK_SOURCES)

################################################################
# Compiler 
################################################################

# You might be used to arm-none-eabi-gcc, but unit tests are run locally!
CC = gcc

################################################################
# C flags 
################################################################

# C Includes
# This should include a path to all of the headers used in your test
C_INCLUDES =                                           \
-I.                                                    \
-I$(ROOT_DIR)/test/framework/src/                      

# Preprocessor definitions
# Don't worry about the defs and flags unless you know what you're doing
C_DEFS =          \
-DUSE_HAL_DRIVER  \
-DUSE_I2C_IT      \
-DSTM32H750xx     \
-DFLIGHT_COMPUTER \
-DA0002_REV2	  \
-DEMULATOR_TEST	  \
-DUNIT_TEST

CFLAGS = $(C_INCLUDES) $(C_DEFS) $(OPT) -Wall -g

# Enable some warnings
CFLAGS += -Wno-pointer-to-int-cast
CFLAGS += -Wno-int-to-pointer-cast
CFLAGS += -Wno-unused-function
CFLAGS += -Wno-unused-variable

# Get coverage report
CFLAGS += -ftest-coverage
CFLAGS += -fprofile-arcs

################################################################
# Build all
################################################################
# Wipe the build folder, then begin compilation
all: clean $(BUILD_DIR)/$(TARGET)

# Setup the path for c files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# Compile
$(BUILD_DIR)/%.o: %.c $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

# Link
$(BUILD_DIR)/$(TARGET): $(OBJECTS) 
	$(CC) $(OBJECTS) -o $@ -lgcov -lm

$(BUILD_DIR):
	mkdir $@		

################################################################
# Build test
################################################################

# Compile test, execute, and generate results
.ONESHELL:
test:
	@echo THIS TEST MUST BE EXECUTED IN A BASH TERMINAL. CMD/PS do not work.
	# Reset Env
	-rm -fR $(BUILD_DIR)
	mkdir -p intermediate_results
	-rm -fR intermediate_results/*
	mkdir -p coverage
	-rm -fR coverage/*

	# Run test suite
	-python ../../../utilities/emulator_runner.py $(ROOT_DIR) $(TEST_SUITE_DIR) $(TEST_SUITE_MOD)

	# Verify results in SDR's format
	$(MAKE) all
	$(BUILD_DIR)/$(TARGET) $(TARGET)
	tail -n 7 "results.txt"

################################################################
# clean up 
################################################################

# Remove build folder
clean:
	-rm -fR $(BUILD_DIR)


################################################################
# EOF 
################################################################