################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../includes/iniparser/ini.c 

OBJS += \
./includes/iniparser/ini.o 

C_DEPS += \
./includes/iniparser/ini.d 


# Each subdirectory must supply rules for building sources it contributes
includes/iniparser/%.o: ../includes/iniparser/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


