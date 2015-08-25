################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include example/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: qed_lcd.axf

# Tool invocations
qed_lcd.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/Users/csweet1/Documents/NXP/Code/lpcopen_2_10_lpcxpresso_ea_devkit_4088/lpc_chip_40xx/Debug" -L"/Users/csweet1/Documents/NXP/Code/lpcopen_2_10_lpcxpresso_ea_devkit_4088/lpc_board_ea_devkit_4088/Debug" -Xlinker -Map="qed_lcd.map" -Xlinker --gc-sections -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -T "periph_sdmmc_Debug.ld" -o "qed_lcd.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) qed_lcd.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "qed_lcd.axf"; # arm-none-eabi-objcopy -v -O binary "qed_lcd.axf" "qed_lcd.bin" ; # checksum -p LPC4088 -d "qed_lcd.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
