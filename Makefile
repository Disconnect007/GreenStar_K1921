.PHONY: clean gdb-server

TARGET = firmware

OPT = -O0

BUILD_DIR = build

C_SOURCES =  \
	src/main.c \
	platform/Device/K1921VG015/source/i2c_tx.c \
	platform/Device/K1921VG015/source/oled_small.c \
	platform/Device/K1921VG015/source/plib015_i2c.c \
	platform/Device/K1921VG015/source/plib015_rcu.c \
	platform/Device/K1921VG015/source/modbus_crc.c \
	platform/Device/K1921VG015/source/modbus_funcs.c \
	platform/Device/K1921VG015/source/uart_tx.c \
	platform/Device/K1921VG015/source/plib015_uart.c \
	platform/Device/K1921VG015/source/mtimer.c \
	platform/Device/K1921VG015/source/plic.c \
	platform/Device/K1921VG015/source/sys_init.c \
	platform/Device/K1921VG015/source/system_k1921vg015.c 

# Неактивные сурсы	platform/Device/K1921VG015/source/i2c_tx.c platform/Device/K1921VG015/source/oled_small.c platform/Device/K1921VG015/source/plib015_i2c.c platform/retarget/Template/K1921VG015/retarget.c \

ASM_SOURCES =  \
	platform/Device/K1921VG015/source/startup_k1921vg015.S

GCC_PATH = /home/aachurakov/K1921_DEV_VSCODE/NIIET_c/K1921VG015/libs/xpack-riscv-none-elf-gcc-15.2.0-1/bin

PREFIX = riscv-none-elf-

OCD_PATH = ../libs/xpack-openocd-k1921vk-0.12.0-k1921vk/bin
OCD = "$(OCD_PATH)/openocd"

CC = "$(GCC_PATH)/$(PREFIX)gcc"
AS = "$(GCC_PATH)/$(PREFIX)gcc" -x assembler-with-cpp
CP = "$(GCC_PATH)/$(PREFIX)objcopy"
SZ = "$(GCC_PATH)/$(PREFIX)size"

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

MCU = -march=rv32imfc -mabi=ilp32f -msmall-data-limit=8 -mstrict-align -mno-save-restore -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -fno-builtin -flto -DSELF_TIMED=1 -Wall -Wextra -g3 -ggdb -DHSECLK_VAL=16000000 -DRETARGET -DSYSCLK_HSE -DCKO_PLL0

C_INCLUDES =  \
	-I. \
	-Isrc \
	-Iplatform/Device/K1921VG015/include \
	-Iplatform/retarget/Template/K1921VG015

ASFLAGS = $(MCU) $(OPT) -MMD -MP -MF"$(@:%.o=%.d)"

CFLAGS = $(MCU) $(C_INCLUDES) $(OPT) -MMD -MP -MF"$(@:%.o=%.d)"

LDSCRIPTS_PATH = platform/Device/K1921VG015/ldscripts
LDSCRIPT = platform/Device/K1921VG015/ldscripts/k1921vg015_flash.ld
LDFLAGS = $(MCU) --specs=nano.specs -lnosys -lgcc -mcmodel=medlow -nostartfiles -ffreestanding $(OPT) -L $(LDSCRIPTS_PATH) -Wl,-Bstatic,-T,$(LDSCRIPT),-Map,$(BUILD_DIR)/$(TARGET).map,--print-memory-usage


all: flash


build: clean $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))


$(BUILD_DIR)/%.o: %.c Makefile | make_folder
	$(CC) -c $(CFLAGS) -g -std=gnu99 -MT"$(@)" -c "$<" -o "$@"

$(BUILD_DIR)/%.o: %.S Makefile | make_folder
	$(AS) -c $(CFLAGS) -g -MT"$(@)" -c "$<" -o "$@"

$(BUILD_DIR)/%.o: %.s Makefile | make_folder
	$(AS) -c $(CFLAGS) -g -MT"$(@)" -c "$<" -o "$@"

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile | make_folder
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | make_folder
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | make_folder
	$(BIN) $< $@

make_folder:
	@-mkdir $(BUILD_DIR)


clean:
	@-rm -rf build


OPENOCD_INTERFACE = ../libs/scripts/interface/vg015_dev_onboard_ftdi.cfg
OPENOCD_TARGET = ../libs/scripts/target/k1921vg015.cfg


flash: erase
#	$(OCD) -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x80000000 verify" -c reset -c exit
#   НЕ ЗАБЫТЬ - я поменял прошивку hex на elf
	$(OCD) -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).elf" -c reset -c exit 


erase: build
	$(OCD) -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c halt -c "flash erase_sector 0 0 last" -c exit


gdb-server:
	$(OCD) -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c "gdb_port 3333"