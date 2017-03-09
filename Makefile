#Target 
#------------------------------------------------------------------------------
TARGET = lukey

#Mcu
#------------------------------------------------------------------------------
MCU = mcu=atmega8

#Defines
#------------------------------------------------------------------------------
DEFINES += F_CPU=8000000UL

#Instuments
#------------------------------------------------------------------------------
AS = avr-gcc
CC = avr-gcc
LD = avr-gcc
CP = avr-objcopy
OD = avr-objdump
SZ = avr-size
RM = rm

#Src dirs
#------------------------------------------------------------------------------
SRCDIRS += 

#Includes
#------------------------------------------------------------------------------
INCLUDES += .
INCLUDES += $(SRCDIRS)

#Settings compiler's
#------------------------------------------------------------------------------
CFLAGS += -Wall 						# -w -Wall -Werror
CFLAGS += -Os 							# -O0 -O1 -O2 -O3 -Os
CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -std=gnu99   					#-ansi -std=c99 -std=gnu99
CFLAGS += -funsigned-char
CFLAGS += -funsigned-bitfields
CFLAGS += $(addprefix -m, $(MCU))
CFLAGS += $(addprefix -D, $(DEFINES))
CFLAGS += $(addprefix -I, $(INCLUDES))
CFLAGS += -g2 -gdwarf-2 				#-g1 -g2 -g3 -gstabs -gdwarf-2  -gstabs+

#Settings linker's
#------------------------------------------------------------------------------
#LDFLAGS += -Wl,-Map,$@.map
# LDFLAGS = -Wl,-Map,$(TARGET).map
LDFLAGS += -Wl,-gc-section
LDFLAGS += $(addprefix -m, $(MCU))

#Assembler's settings
#------------------------------------------------------------------------------
AFLAGS += -x assembler-with-cpp
AFLAGS += $(addprefix -m, $(MCU))
AFLAGS += -g2

#Obj files
#------------------------------------------------------------------------------
OBJS += $(patsubst %.c, %.o, $(wildcard *.c))
OBJS += $(patsubst %.c, %.o, $(wildcard $(addsuffix /*.c ,$(SOURCEDIRS))))

#List files for remove
#------------------------------------------------------------------------------
TOREMOVE += *.elf *.hex *.o *.d *.eep *.lss
TOREMOVE += $(addsuffix /*.o ,$(SOURCEDIRS))
TOREMOVE += $(addsuffix /*.d ,$(SOURCEDIRS))

#Secondary outputs
#------------------------------------------------------------------------------
LSS += $(TARGET).lss
EEPROM_IMAGE += $(TARGET).eep
SIZEDUMMY += sizedummy

.PHONY : all clean

all: $(TARGET).hex secondary-outputs

$(TARGET).hex: $(TARGET).elf
	$(CP) -j .text -j .data -Oihex $(TARGET).elf $(TARGET).hex

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@  $^

%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC)  $(CFLAGS) -MD -c $< -o $@

%.o: %.s 
	$(AS) $(AFLAGS) -c $< -o $@

$(TARGET).lss: $(TARGET).elf
	$(OD) -h -S $(TARGET).elf > $(TARGET).lss	

$(TARGET).eep: $(TARGET).elf
	$(CP) -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -Oihex $(TARGET).elf $(TARGET).eep

sizedummy: $(TARGET).elf 
	$(SZ) --format=avr   --$(MCU) $(TARGET).elf

include $(wildcard *.d)

clean: 
	@$(RM) -f $(TOREMOVE) 

secondary-outputs: $(LSS) $(EEPROM_IMAGE) $(SIZEDUMMY)
