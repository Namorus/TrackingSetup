################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../armadillo/armadillo_files/src/wrapper.cpp 

OBJS += \
./armadillo/armadillo_files/src/wrapper.o 

CPP_DEPS += \
./armadillo/armadillo_files/src/wrapper.d 


# Each subdirectory must supply rules for building sources it contributes
armadillo/armadillo_files/src/%.o: ../armadillo/armadillo_files/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


