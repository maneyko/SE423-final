################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c2000_17.6.0.STS

GEN_OPTS__FLAG := --cmd_file="configPkg/compiler.opt"
GEN_CMDS__FLAG := -l"configPkg/linker.cmd"

ORDERED_OBJS += \
"./28335_dma.obj" \
"./28335_eqep.obj" \
"./28335_inits.obj" \
"./28335_mcbsp.obj" \
"./28335_pwm.obj" \
"./28335_serial.obj" \
"./28335_spi.obj" \
"./DSP2833x_ADC_cal.obj" \
"./DSP2833x_Adc.obj" \
"./DSP2833x_CSMPasswords.obj" \
"./DSP2833x_CodeStartBranch.obj" \
"./DSP2833x_CpuTimers.obj" \
"./DSP2833x_GlobalVariableDefs.obj" \
"./DSP2833x_Mcbsp.obj" \
"./DSP2833x_PieCtrl.obj" \
"./DSP2833x_Spi.obj" \
"./DSP2833x_SysCtrl.obj" \
"./DSP2833x_usDelay.obj" \
"./i2c.obj" \
"./lcd.obj" \
"./user_lab3_1.obj" \
"C:/dgnava2_maneyko2/SE423Repo/Labs/lab3_1/v110/DSP2833x_headers/cmd/DSP2833x_Headers_BIOS.cmd" \
"../TMS320F28335.cmd" \
"C:/dgnava2_maneyko2/SE423Repo/Labs/lab3_1/cmd/user_lnk.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
lab3_1.out \

EXE_OUTPUTS__QUOTED += \
"lab3_1.out" \

BIN_OUTPUTS += \
lab3_1.hex \

BIN_OUTPUTS__QUOTED += \
"lab3_1.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "lab3_1.out"

# Tool invocations
lab3_1.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c2000_17.6.0.STS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --define=LARGE_MODEL --define=_DEBUG --define=DSP28_BIOS --define=F28335_CONTROL_CARD --define=F28335_CONTROL_CARD30 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on -z -m"lab3_1.map" --stack_size=0x300 --warn_sections -i"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c2000_17.6.0.STS/lib" -i"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c2000_17.6.0.STS/include" --reread_libs --diag_wrap=off --display_error_number --xml_link_info="lab3_1_linkInfo.xml" --rom_model -o "lab3_1.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

lab3_1.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: C2000 Hex Utility'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c2000_17.6.0.STS/bin/hex2000"  -o "lab3_1.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_CMDS__QUOTED)$(GEN_MISC_FILES__QUOTED)$(BIN_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)$(GEN_OPTS__QUOTED)
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-$(RM) "28335_dma.obj" "28335_eqep.obj" "28335_inits.obj" "28335_mcbsp.obj" "28335_pwm.obj" "28335_serial.obj" "28335_spi.obj" "DSP2833x_ADC_cal.obj" "DSP2833x_Adc.obj" "DSP2833x_CSMPasswords.obj" "DSP2833x_CodeStartBranch.obj" "DSP2833x_CpuTimers.obj" "DSP2833x_GlobalVariableDefs.obj" "DSP2833x_Mcbsp.obj" "DSP2833x_PieCtrl.obj" "DSP2833x_Spi.obj" "DSP2833x_SysCtrl.obj" "DSP2833x_usDelay.obj" "i2c.obj" "lcd.obj" "user_lab3_1.obj" 
	-$(RM) "28335_dma.d" "28335_eqep.d" "28335_inits.d" "28335_mcbsp.d" "28335_pwm.d" "28335_serial.d" "28335_spi.d" "DSP2833x_Adc.d" "DSP2833x_CpuTimers.d" "DSP2833x_GlobalVariableDefs.d" "DSP2833x_Mcbsp.d" "DSP2833x_PieCtrl.d" "DSP2833x_Spi.d" "DSP2833x_SysCtrl.d" "i2c.d" "lcd.d" "user_lab3_1.d" 
	-$(RM) "DSP2833x_ADC_cal.d" "DSP2833x_CSMPasswords.d" "DSP2833x_CodeStartBranch.d" "DSP2833x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

