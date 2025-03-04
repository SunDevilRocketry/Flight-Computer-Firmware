################################################################ 
#
# gps.c unit tests (based on gcc)
#
################################################################


################################################################
# target
################################################################
TARGET = test_gps


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
ROOT_DIR  = ../../..

# library directory
LIB_DIR = $(ROOT_DIR)/lib

INIT_DIR = $(ROOT_DIR)/init


################################################################
# source 
################################################################

# C sources
C_SOURCES =                                   						  \
test_gps.c                       								  \
mock/MOCK_hal.c													  \
$(ROOT_DIR)/mod/gps/gps.c           							  \
$(ROOT_DIR)/Unity/src/unity.c    

# Tested Source Files
COV_C_SOURCES =														\
$(ROOT_DIR)/mod/gps/gps.c

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
-I$(ROOT_DIR)/mod/gps/								   \
-I$(ROOT_DIR)/mod/sensor/							   \
-I$(ROOT_DIR)/mod/imu/								   \
-I$(LIB_DIR)                                           \
-I$(LIB_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc          \
-I$(LIB_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy   \
-I$(LIB_DIR)/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
-I$(LIB_DIR)/Drivers/CMSIS/Include                     \
-I$(ROOT_DIR)/init/terminal/config/inc				   \
-I$(INIT_DIR)/config/Inc                               \
-I$(ROOT_DIR)/Unity/src/                               \
-I$(ROOT_DIR)/test/mod/gps/mock/

# Preprocessor definitions
C_DEFS =         \
-DUSE_HAL_DRIVER \
-DSTM32H750xx    \
-DFLIGHT_COMPUTER \
-DUNIT_TEST

CFLAGS = $(C_INCLUDES) $(C_DEFS) $(OPT) -Wall -g

# Disable some warnings
CFLAGS += -Wno-pointer-to-int-cast
CFLAGS += -Wno-int-to-pointer-cast
CFLAGS += -Wno-unused-function
CFLAGS += -Wno-unused-variable

# Get coverage report
CFLAGS += -ftest-coverage
CFLAGS += -fprofile-arcs

# build all 
all: $(BUILD_DIR)/$(TARGET)


################################################################
# Build the application 
################################################################

# Setup the path for c files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

$(BUILD_DIR)/%.o: %.c $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET): $(OBJECTS) 
	$(CC) $(OBJECTS) -o $@ -lgcov

$(BUILD_DIR):
	mkdir $@		

test: $(BUILD_DIR)/$(TARGET)
	"$(BUILD_DIR)/$(TARGET).exe" > "$(TARGET)_results.txt"
	gcov -o $(BUILD_DIR) $(COV_C_SOURCES)

################################################################
# clean up 
################################################################

clean:
	-rm -fR $(BUILD_DIR)


################################################################
# EOF 
################################################################