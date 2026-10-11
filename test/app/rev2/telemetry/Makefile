################################################################
#
# telemetry.c unit tests (based on gcc)
#
################################################################

TARGET = telemetry

################################################################
# build variables
################################################################

DEBUG ?= 0
OPT   = -Og

################################################################
# paths
################################################################

BUILD_DIR = build
ROOT_DIR  = ../../../..
FRAMEWORK_DIR = ../../../framework

LIB_DIR = $(ROOT_DIR)/lib
DRIVER_DIR = $(ROOT_DIR)/driver

################################################################
# source
################################################################

TEST_SOURCES = \
test_telemetry.c \
mock/test_telemetry_stubs.c

COV_C_SOURCES = \
$(ROOT_DIR)/app/rev2/telemetry_contract.c

FRAMEWORK_SOURCES = \
$(FRAMEWORK_DIR)/src/test_assert.c \
$(FRAMEWORK_DIR)/src/test_runner.c

C_SOURCES = $(TEST_SOURCES) $(COV_C_SOURCES) $(FRAMEWORK_SOURCES)

################################################################
# compiler
################################################################

CC = gcc

################################################################
# C flags
################################################################

C_INCLUDES = \
-I. \
-I./mock \
-I$(ROOT_DIR)/app/rev2 \
-I$(ROOT_DIR)/mod/telemetry \
-I$(ROOT_DIR)/mod/debug_sdr \
-I$(ROOT_DIR)/mod/commands \
-I$(ROOT_DIR)/mod/math_sdr \
-I$(ROOT_DIR)/mod/error_sdr \
-I$(ROOT_DIR)/mod/sensor \
-I$(ROOT_DIR)/driver/imu \
-I$(ROOT_DIR)/driver/gps \
-I$(ROOT_DIR)/driver/baro \
-I$(ROOT_DIR)/driver/flash \
-I$(ROOT_DIR)/driver/usb \
-I$(ROOT_DIR)/driver/buzzer \
-I$(ROOT_DIR)/driver/led \
-I$(ROOT_DIR)/driver/ignition \
-I$(ROOT_DIR)/driver/servo \
-I$(ROOT_DIR)/driver/lora \
-I$(LIB_DIR) \
-I$(LIB_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc \
-I$(LIB_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy \
-I$(LIB_DIR)/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
-I$(LIB_DIR)/Drivers/CMSIS/Include \
-I$(ROOT_DIR)/init/rev2/config/Inc \
-I$(FRAMEWORK_DIR)/src

C_DEFS = \
-DUSE_HAL_DRIVER \
-DUSE_I2C_IT \
-DSTM32H750xx \
-DFLIGHT_COMPUTER \
-DA0002_REV2 \
-DUNIT_TEST

CFLAGS = $(C_INCLUDES) $(C_DEFS) $(OPT) -Wall -g
CFLAGS += -Wno-pointer-to-int-cast
CFLAGS += -Wno-int-to-pointer-cast
CFLAGS += -Wno-unused-function
CFLAGS += -Wno-unused-variable
CFLAGS += -Wno-implicit-function-declaration
CFLAGS += -ftest-coverage
CFLAGS += -fprofile-arcs
CFLAGS += -ffile-prefix-map=$(shell pwd)/=

ifeq ($(DEBUG), 1)
CFLAGS += -DDEBUG
else
CFLAGS += -DRELBLD
endif

################################################################
# build all
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
# build test
################################################################

test:
	@echo THIS TEST MUST BE EXECUTED IN A BASH TERMINAL. CMD/PS do not work.
	-rm -fR $(BUILD_DIR)
	$(MAKE) all
	$(BUILD_DIR)/$(TARGET)
	tail -n 7 "results.txt"
	mkdir -p coverage
	gcovr $(BUILD_DIR) \
		--filter $(COV_C_SOURCES) \
		--html-details coverage/coverage.html \
		--json coverage/coverage.json

################################################################
# clean up
################################################################

clean:
	-rm -fR $(BUILD_DIR)
