################################################################
#
# fin_calib unit tests (based on gcc)
#
################################################################


################################################################
# target
################################################################
TARGET = fin_calib

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
DRIVER_DIR = $(ROOT_DIR)/driver
INIT_DIR = $(ROOT_DIR)/init

################################################################
# source 
################################################################

# C sources (for test. driver, stubs, and others should go here)
TEST_SOURCES =                                   				  \
test_fin_calib.c                       					          \
mock/MOCK_hal.c													  \
mock/MOCK_usb.c                                                   \
mock/MOCK_led.c 												  \
mock/MOCK_servo.c                                                 \

# Tested Source Files (anything that is being tested for coverage)
COV_C_SOURCES =													  \
$(ROOT_DIR)/app/rev2/fin_calib.c 

# Framework (just these files. do not modify.)
FRAMEWORK_SOURCES =												  \
$(ROOT_DIR)/test/framework/src/test_assert.c          	  		  \
$(ROOT_DIR)/test/framework/src/test_runner.c

C_SOURCES = $(TEST_SOURCES) $(COV_C_SOURCES) $(FRAMEWORK_SOURCES)

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
-I$(ROOT_DIR)/app/rev2/					   			   \
-I$(DRIVER_DIR)/gps/								   \
-I$(ROOT_DIR)/mod/sensor/							   \
-I$(DRIVER_DIR)/imu/								   \
-I$(DRIVER_DIR)/lora								   \
-I$(LIB_DIR)                                           \
-I$(LIB_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc          \
-I$(LIB_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy   \
-I$(LIB_DIR)/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
-I$(LIB_DIR)/Drivers/CMSIS/Include                     \
-I$(ROOT_DIR)/init/rev2/config/Inc				   	   \
-I$(ROOT_DIR)/test/framework/src                       \
-I$(ROOT_DIR)/test/app/rev2/fin_calib/mock	   	       \
-I$(ROOT_DIR)/mod/math_sdr							   \
-I$(DRIVER_DIR)/servo								   \
-I$(DRIVER_DIR)/usb								   	   \
-I$(DRIVER_DIR)/flash								   \
-I$(ROOT_DIR)/mod/sensor							   \
-I$(DRIVER_DIR)/imu								       \
-I$(DRIVER_DIR)/led     							   \
-I$(ROOT_DIR)/lib/util

# Preprocessor definitions
# Don't worry about the defs and flags unless you know what you're doing
C_DEFS =         \
-DUSE_HAL_DRIVER \
-DSTM32H750xx    \
-DFLIGHT_COMPUTER \
-DUNIT_TEST       \
-DA0002_REV2 

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

# Remove build folder
clean:
	-rm -fR $(BUILD_DIR)


################################################################
# EOF 
################################################################