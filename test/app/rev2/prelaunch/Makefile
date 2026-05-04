################################################################
#
# prelaunch unit tests (based on gcc)
#
################################################################

################################################################
# target
################################################################
TARGET = prelaunch


################################################################
# build variables 
################################################################

# Debug build?
DEBUG = 0
# Optimization
OPT   = -Og


################################################################
# paths 
################################################################

# build folder
BUILD_DIR = build

# base directory
# depending on your test, this might not be correct.
ROOT_DIR  = ../../../..

# directory shortcuts for includes and sources
LIB_DIR = $(ROOT_DIR)/lib
MOD_DIR = $(ROOT_DIR)/mod


################################################################
# source 
################################################################

# C sources
TEST_SOURCES =   												\
mock/led.c														\
test_prelaunch.c												\
mock/sensor.c													\
mock/MOCK_hal.c													\
mock/usb.c														\
mock/fin_calib.c												\
mock/ignition.c													\
mock/servo.c													\
mock/mocks.c													\
mock/common.c													\
mock/MOCK_flash.c													\
mock/mock_huart.c

# Tested Source Files
COV_C_SOURCES =													  \
$(ROOT_DIR)/app/rev2/prelaunch.c 

# Framework
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
-I$(ROOT_DIR)/app/rev2/					   \
-I$(ROOT_DIR)/driver/gps/								   \
-I$(ROOT_DIR)/mod/sensor/							   \
-I$(ROOT_DIR)/driver/imu/								   \
-I$(LIB_DIR)                                           \
-I$(LIB_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc          \
-I$(LIB_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy   \
-I$(LIB_DIR)/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
-I$(LIB_DIR)/Drivers/CMSIS/Include                     \
-I$(ROOT_DIR)/init/rev2/config/Inc				   \
-I$(ROOT_DIR)/test/framework/src                       \
-I$(ROOT_DIR)/test/app/rev2/prelaunch/mock	   	   \
-I$(ROOT_DIR)/driver/buzzer								\
-I$(ROOT_DIR)/mod/commands								\
-I$(ROOT_DIR)/driver/servo								   \
-I$(ROOT_DIR)/driver/ignition								   \
-I$(ROOT_DIR)/driver/flash								   \
-I$(ROOT_DIR)/mod/sensor							   \
-I$(ROOT_DIR)/driver/imu								   \
-I$(ROOT_DIR)/lib/util									\
-I$(ROOT_DIR)/driver/led								   	\
-I$(ROOT_DIR)/driver/lora								   	\
-I$(ROOT_DIR)/mod/common								\
-I$(ROOT_DIR)/mod/error_sdr								\
-I$(ROOT_DIR)/mod/sensor								\
-I$(ROOT_DIR)/mod/math_sdr								\
-I$(ROOT_DIR)/driver/usb

# Preprocessor definitions
# Don't worry about the defs and flags unless you know what you're doing
C_DEFS =         \
-DUSE_HAL_DRIVER \
-DSTM32H750xx    \
-DFLIGHT_COMPUTER \
-DUNIT_TEST	\
-DA0002_REV2

CFLAGS = $(C_INCLUDES) $(C_DEFS) $(OPT) -Wall -g

# Disable some warnings
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

# Setup the path for c files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
# This is to make sure fin_calib.c is excluded 
vpath %.c ./mock
vpath %.c $(ROOT_DIR)/app/rev2
vpath %.c $(ROOT_DIR)/test/framework/src

$(BUILD_DIR)/%.o: %.c $(BUILD_DIR)
	$(CC) -c -w $(CFLAGS) $< -o $@

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