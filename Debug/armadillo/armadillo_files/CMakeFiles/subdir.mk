################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../armadillo/armadillo_files/CMakeFiles/feature_tests.cxx 

OBJS += \
./armadillo/armadillo_files/CMakeFiles/feature_tests.o 

CXX_DEPS += \
./armadillo/armadillo_files/CMakeFiles/feature_tests.d 


# Each subdirectory must supply rules for building sources it contributes
armadillo/armadillo_files/CMakeFiles/%.o: ../armadillo/armadillo_files/CMakeFiles/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


