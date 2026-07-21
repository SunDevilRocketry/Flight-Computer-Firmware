################################################################
#
# math_sdr unit tests (based on gcc)
#
################################################################

################################################################
# target
################################################################
TARGET = math_sdr

################################################################
# build variables
################################################################
DEBUG ?= 0
OPT = -Og

################################################################
# paths
################################################################
BUILD_DIR = build
ROOT_DIR = ../../../..
MOD_DIR = $(ROOT_DIR)/mod

################################################################
# source
################################################################
TEST_SOURCES = \
test_math_sdr.c

COV_C_SOURCES = \
$(MOD_DIR)/math_sdr/math_sdr.c

FRAMEWORK_SOURCES = \
$(ROOT_DIR)/test/framework/src/test_assert.c \
$(ROOT_DIR)/test/framework/src/test_runner.c

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
-I$(MOD_DIR)/math_sdr \
-I$(ROOT_DIR)/test/framework/src

C_DEFS = \
-DUNIT_TEST

CFLAGS = $(C_INCLUDES) $(C_DEFS) $(OPT) -Wall -g

CFLAGS += -Wno-unused-function
CFLAGS += -Wno-unused-variable
CFLAGS += -ftest-coverage
CFLAGS += -fprofile-arcs
ifeq ($(DEBUG), 1)
CFLAGS += -DDEBUG
else
CFLAGS += -DRELBLD
endif

################################################################
# build
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
# test
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
# clean
################################################################
clean:
	-rm -fR $(BUILD_DIR)
