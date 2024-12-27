# Directories
# Since avr-gcc was installed using apt, the default 
# directories are included in the global environment 
# variables. So no directories specified here.
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj
BIN_DIR = $(BUILD_DIR)/bin
SRC_DIR = src
DRIVERS_DIR = $(SRC_DIR)/drivers
COMMON_DIR = $(SRC_DIR)/common
APP_DIR = $(SRC_DIR)/app
EXTERNAL_DIR = external

# Toolchain
CC = avr-gcc

# Files
TARGET = $(BIN_DIR)/h_bridge
SOURCES = main.c
OBJECT_NAMES = $(SOURCES:.c=.o)
OBJECTS = $(patsubst %,$(OBJ_DIR)/%,$(OBJECT_NAMES))

# Flags
MCU = attiny2313
WFLAGS = -Wall -Wextra -Werror -Wshadow
#ASMFLAGS = -Wa,-ahlmns=$(TARGET).lst
ASMFLAGS = -Wa,-ahlmns=$(patsubst %.o,%.lst,$(OBJECTS))
OPTIMIZATIONFLAGS = -Os
DEBUGFLAGS = -g
CFLAGS = $(WFLAGS) -mmcu=$(MCU) $(ASMFLAGS) $(OPTIMIZATIONFLAGS)
LDFLAGS = $(WFLAGS) -mmcu=$(MCU) $(OPTIMIZATIONFLAGS)

AVRDUDE_MCU = t2313
AVRDUDE_PROGRAMMER = usbasp
AVRDUDE_MCU_FLAG = -p $(AVRDUDE_MCU)
AVRDUDE_PROGRAMMER_FLAG = -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FILE_TO_WRITE_FLAG = -U flash:w:$(BIN_DIR)/$(TARGET).hex:i
AVRDUDE_COMMON_FLAGS = -B4


# Converting elf to hex
$(TARGET): $(TARGET).elf
	avr-objcopy -j .text -j .data -O ihex $^ $@.hex

# Linking
$(TARGET).elf: $(OBJECTS)
	mkdir -p $(dir $@)
	avr-gcc -DF_CPU=4000000 $(LDFLAGS) -o $@ $^

# Compiling
# -c flag means compile only no linking
# -Os flag means optimization for the size 
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	mkdir -p $(dir $@)
	avr-gcc $(CFLAGS) -c -o $@ $^
#	avr-gcc -g -Wall -Os -Werror -Wextra -mmcu=attiny2313 -c -o h_bridge.o main.c 
#	avr-gcc -g -DF_CPU=1000000UL -Wall -Os -Werror -Wextra -mmcu=attiny2313 Wa,-ahlmns=h_bridge.lst -c -o h_bridge.o main.c


.PHONY: all clean flash

all: $(TARGET)

clean: 
	$(RM) -r $(BUILD_DIR)

flash: $(TARGET)
	avrdude $(AVRDUDE_PROGRAMMER_FLAG) $(AVRDUDE_MCU_FLAG) $(AVRDUDE_FILE_TO_WRITE_FLAG) $(AVRDUDE_COMMON_FLAGS)
#	avrdude -c usbasp -p t2313 -U flash:w:$(BIN_DIR)/$(TARGET).hex:i -B4
