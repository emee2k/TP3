################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include freertos/src/subdir.mk
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
all: TP3.axf

# Tool invocations
TP3.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\emili\OneDrive\Documents\Sistemas embebidos\workspace-SE-2018- TPs\lpc_chip_43xx\Debug" -L"C:\Users\emili\OneDrive\Documents\Sistemas embebidos\workspace-SE-2018- TPs\lpc_board_nxp_lpcxpresso_4337\Debug" -Xlinker -Map="TP3.map" -Xlinker --gc-sections -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -T "freertos_examples_1_to_9_Debug.ld" -o "TP3.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) TP3.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "TP3.axf"; # arm-none-eabi-objcopy -v -O binary "TP3.axf" "TP3.bin" ; # checksum -p LPC4337 -d "TP3.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
