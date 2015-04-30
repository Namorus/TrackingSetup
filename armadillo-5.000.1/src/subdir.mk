################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../armadillo-5.000.1/src/wrapper.cpp 

OBJS += \
./armadillo-5.000.1/src/wrapper.o 

CPP_DEPS += \
./armadillo-5.000.1/src/wrapper.d 


# Each subdirectory must supply rules for building sources it contributes
armadillo-5.000.1/src/%.o: ../armadillo-5.000.1/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


