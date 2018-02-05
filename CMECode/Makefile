#
# PMD CME template makefile
# portions by Martin Thomas, Kaiserslautern, Germany 
#
#
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# To rebuild project do "make clean" then "make all".
#

CMOTIONDIR = $(OFFSETDIR)../c-motion
# List C source files here. (C dependencies are automatically generated.)
# use file-extension c for "c-only"-files
SRC  = $(TARGET).c
SRC += $(PROJECTFILES)

SRC += $(CMOTIONDIR)/c/c-motion.c
SRC += $(CMOTIONDIR)/c/PMDtrans.c
SRC += $(CMOTIONDIR)/c/PMDdiag.c
SRC += $(CMOTIONDIR)/c/PMDcommon.c
SRC += $(CMOTIONDIR)/c/PMDMB.c
SRC += $(CMOTIONDIR)/c/PMDutil.c
SRC += $(OFFSETDIR)syscalls.c
SRC += $(OFFSETDIR)startup.c

# List Assembler source files here.
ASRC = $(OFFSETDIR)UserImport.S

# List directories in which to search for source files.
# With this feature source file names must be unique.
# Note that pathnames with embedded spaces will have problems.
VPATH = $(OFFSETDIR) $(CMOTIONDIR)/c

# CME preprocessor definition
CDEFS =  -DCME

# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
EXTRAINCDIRS  = $(OFFSETDIR) 
EXTRAINCDIRS += $(CMOTIONDIR)/include

# Path to Linker-Scripts
LINKERSCRIPTPATH = $(OFFSETDIR)

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
OPT = 3

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

# Compiler flags to generate dependency files.
GENDEPFLAGS = -MD -MP -MF dep/$(@F).d

# Compiler flags.
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#
# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)
CFLAGS += -O$(OPT)
CFLAGS += -mcpu=$(MCU) 
CFLAGS += $(CDEFS) $(CINCS) -I.
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wall -Wcast-align -Wimplicit
CFLAGS += -Wpointer-arith -Wswitch
CFLAGS += $(GENDEPFLAGS)
CFLAGS += -Wcast-qual

# flags only for C
CONLYFLAGS += -Wnested-externs 
CONLYFLAGS += $(CSTANDARD)

# Assembler flags.
#  -Wa,...:    tell GCC to pass this to the assembler.
#  -ahlns:     create listing
#  -g$(DEBUG): have the assembler create line number information
ASFLAGS = $(ADEFS)
ASFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))


# Additional libraries.

MATH_LIB = -lm

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -nostartfiles
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref,--gc-sections
#LDFLAGS += -nostdlib
LDFLAGS += $(MATH_LIB)
#LDFLAGS += -v
LDFLAGS += -lc
#LDFLAGS += -L$(LINKERSCRIPTPATH)
#LDFLAGS += -lpmd
LDFLAGS += -T$(LINKERSCRIPTPATH)/linkPMD.ld

TCHAIN = arm-elf
MCU      = arm9

# Define directories, if needed.
DIRARM = $(PATH_GNU_ARM)
#DIRARMBIN = $(DIRARM)/bin/
#DIRARMUTILS = $(DIRARM)/utils/bin/

# Define programs and commands.
CC        = $(DIRARMBIN)$(TCHAIN)-gcc
AR        = $(DIRARMBIN)$(TCHAIN)-ar
OBJCOPY   = $(DIRARMBIN)$(TCHAIN)-objcopy
OBJDUMP   = $(DIRARMBIN)$(TCHAIN)-objdump
SIZE      = $(DIRARMBIN)$(TCHAIN)-size
NM        = $(DIRARMBIN)$(TCHAIN)-nm
SHELL     = $(DIRARMUTILS)sh
COPY      = $(DIRARMUTILS)cp
REMOVE    = $(DIRARMUTILS)rm -f
REMOVEDIR = $(DIRARMUTILS)rm -f -r
MAKEDIR   = $(DIRARMUTILS)mkdir

# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size:
MSG_FLASH = Creating CME flash file:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_LIBBING = Creating lib file:
MSG_COMPILING = Compiling C:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:

# Define all object files.  Note that these will be put in the directory "obj".
COBJ      = $(addprefix obj/, $(notdir $(SRC:.c=.o)))
AOBJ      = $(addprefix obj/, $(notdir $(ASRC:.S=.o)))
#OBJ       = $(wildcard *.o)

# Define all listing files.
LST = $(ASRC:.S=.lst) $(SRC:.c=.lst) 

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS  = 
ALL_ASFLAGS = -mcpu=$(MCU) -I. $(ASFLAGS)

# Default target.
all: begin obj build finished end
lib: obj begin lib finished end

#build: elf bin lss sym
lib: elf a
build: elf bin
bin: $(TARGET).bin
elf: $(TARGET).elf
lss: $(TARGET).lss 
sym: $(TARGET).sym
a: libPMD.a

# Ensure directory exists
obj:
	$(MAKEDIR) obj
	$(MAKEDIR) dep

# Messages.
begin:
	@echo
	@echo $(MSG_BEGIN)

finished:
	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)
	@echo


#SIZEOPTIONS = -A
# Display sizes of sections.
ELFSIZE = $(SIZE) $(SIZEOPTIONS) $(TARGET).elf
sizebefore:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi

# Display compiler version information.
gccversion : 
	@$(CC) --version

# Create final output file (.bin) from ELF output file.
%.bin: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O binary $< $@

# Create extended listing file from ELF output file.
# testing: option -C
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -C $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(AOBJ) $(COBJ)
%.elf:  $(AOBJ) $(COBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(CFLAGS) $(AOBJ) $(COBJ)  --output $@ $(LDFLAGS)

%.a:    $(AOBJ) $(COBJ)
	@echo
	@echo $(MSG_LIBBING) $@
	$(AR) rcs $@ $(AOBJ) $(COBJ)

# Compile: create object files from C source files.
$(COBJ): obj/%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(CFLAGS) $(CONLYFLAGS) $< -o $@ 

# Assemble: create object files from assembler source files.
$(AOBJ): obj/%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@


# Target: clean project.
clean: begin clean_list finished end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(TARGET).bin  
	$(REMOVE) $(TARGET).elf  
	$(REMOVE) $(TARGET).map  
	$(REMOVE) $(TARGET).sym  
	$(REMOVE) $(TARGET).lnk  
	$(REMOVE) $(TARGET).lss  
	$(REMOVE) $(COBJ)
	$(REMOVE) $(AOBJ)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVEDIR) obj
	$(REMOVEDIR) dep


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex bin lss sym clean clean_list

