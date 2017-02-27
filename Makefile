#######################
# SDCC Makefile for making a hexfile from all .C files in this directory.
# Hexfile location is '.', other output files are generated in directory 'output'.
#######################
#
# From https://github.com/gicking/STM8_templates.git


DEVICE=STM8S003

OPTIMIZE = --opt-code-size

# define compiler path (if not in PATH), and flags
CC       = sdcc
CFLAGS   = -mstm8 --std-sdcc99 -D$(DEVICE) -DUART1_FIFO
LFLAGS   = -mstm8 -lstm8 $(OPTIMIZE) --out-fmt-ihx

# set target
OBJDIR   = $(DEVICE)
TARGET   = $(OBJDIR)/hc12pkt.ihx

# set project options (all .c files in folder)
PRJ_ROOT = .
PRJ_INC_DIR = $(PRJ_ROOT)
PRJ_SRC_DIR = $(PRJ_ROOT)
PRJ_SOURCE  = $(wildcard *.c)
PRJ_HEADER  = $(wildcard *.h)
PRJ_OBJECT  = $(addprefix $(OBJDIR)/, $(PRJ_SOURCE:.c=.rel))

# set STM8_Lib options
LIB_ROOT    = ./STM8_Lib
LIB_INC_DIR = $(LIB_ROOT)
LIB_SRC_DIR = $(LIB_ROOT)
LIB_SOURCE  = gpio.c uart1.c putchar.c
LIB_HEADER  = $(patsubst %.c,%.h,$(LIB_SOURCE))
LIB_OBJECT := $(addprefix $(OBJDIR)/, $(LIB_SOURCE:.c=.rel))

# concat all include folders
INCLUDE = -I$(PRJ_INC_DIR) -I$(LIB_INC_DIR)

# concat all source directories
VPATH=$(PRJ_SRC_DIR):$(LIB_SRC_DIR)


.PHONY: clean all default objects

.PRECIOUS: $(TARGET) $(PRJ_OBJECT) $(LIB_OBJECT) 

default: $(OBJDIR) $(TARGET)

all: default

# create output folder
$(OBJDIR):
	mkdir -p $(OBJDIR)

# compile all *c files
$(OBJDIR)/%.rel: %.c $(PRJ_HEADER) $(LIB_HEADER) $(LIB_SOURCE) $(OBJDIR)
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

# link all object files and libaries
$(TARGET): $(PRJ_OBJECT) $(LIB_OBJECT)
	$(CC) $(LFLAGS) $(PRJ_OBJECT) $(LIB_OBJECT) -o $@
	
# clean up
clean:
	rm -fr .DS_Store
	rm -fr -- -p
	rm -fr $(OBJDIR)
	rm -fr *.TMP

# upload via stm8flash / SWIM (https://github.com/vdudouyt/stm8flash)
# stm8flash requires device in lower-case -> http://gnu-make.2324884.n4.nabble.com/how-to-achieve-conversion-to-uppercase-td14496.html 
swim:
	stm8flash -c stlinkv2 -w $(TARGET) -p stm8s003p3 

