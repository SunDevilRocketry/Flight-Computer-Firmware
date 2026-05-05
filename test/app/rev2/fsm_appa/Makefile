################################################################
#
# fsm_appa.c unit tests (based on gcc)
#
################################################################



################################################################
# target
################################################################
TARGET = fsm_appa


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
ROOT_DIR  = ../../../..

# directory shortcuts for includes and sources
LIB_DIR  = $(ROOT_DIR)/lib
MOD_DIR  = $(ROOT_DIR)/mod
INIT_DIR = $(ROOT_DIR)/init
DRIVER_DIR = $(ROOT_DIR)/driver



################################################################
# source 
################################################################

# C sources (for test. driver, stubs, and others should go here)
TEST_SOURCES =                                \
test_fsm_appa.c                               \
mock/test_fsm_appa_stubs.c

# Tested Source Files (anything that is being tested for coverage)
COV_C_SOURCES =                               \
$(ROOT_DIR)/app/rev2/fsm_appa.c

# Framework (just these files. do not modify.)
FRAMEWORK_SOURCES =                           \
$(ROOT_DIR)/test/framework/src/test_assert.c  \
$(ROOT_DIR)/test/framework/src/test_runner.c

C_SOURCES = $(TEST_SOURCES) $(COV_C_SOURCES) $(FRAMEWORK_SOURCES)



################################################################
# Compiler 
################################################################

CC = gcc



################################################################
# C flags 
################################################################

# C Includes
C_INCLUDES =                                           \
-I.                                                    \
-I./mock                                               \
-I$(ROOT_DIR)/app/rev2/                				   \
-I$(ROOT_DIR)/driver/gps/                                 \
-I$(ROOT_DIR)/driver/buzzer/                              \
-I$(MOD_DIR)/math_sdr/                              \
-I$(MOD_DIR)/sensor/                              \
-I$(DRIVER_DIR)/ignition/                            \
-I$(DRIVER_DIR)/flash/                               \
-I$(DRIVER_DIR)/servo/                               \
-I$(DRIVER_DIR)/usb/                                 \
-I$(DRIVER_DIR)/lora								   \
-I$(DRIVER_DIR)/led/                                 \
-I$(DRIVER_DIR)/imu/                                 \
-I$(MOD_DIR)/error_sdr/                           \
-I$(LIB_DIR)                                           \
-I$(ROOT_DIR)/lib/util                                 \
-I$(LIB_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc          \
-I$(LIB_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy   \
-I$(LIB_DIR)/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
-I$(LIB_DIR)/Drivers/CMSIS/Include                     \
-I$(INIT_DIR)/rev2/config/Inc							\
-I$(ROOT_DIR)/test/framework/src/

# Preprocessor definitions
C_DEFS =         \
-DUSE_HAL_DRIVER \
-DSTM32H750xx    \
-DFLIGHT_COMPUTER \
-DA0002_REV2     \
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

all: clean $(BUILD_DIR)/$(TARGET)

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

$(BUILD_DIR)/%.o: %.c $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ -lgcov -lm

$(BUILD_DIR):
	mkdir $@



################################################################
# Build test
################################################################

test:
	@echo THIS TEST MUST BE EXECUTED IN A BASH TERMINAL. CMD/PS do not work.
	-rm -fR $(BUILD_DIR)
	$(MAKE) all
	$(BUILD_DIR)/$(TARGET)
	tail -n 7 "results.txt"
	mkdir -p coverage
	gcovr --filter $(COV_C_SOURCES) --html-details coverage/coverage.html



################################################################
# clean up 
################################################################

clean:
	-rm -fR $(BUILD_DIR)



################################################################
# EOF 
################################################################
