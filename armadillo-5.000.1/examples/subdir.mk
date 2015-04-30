################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../armadillo-5.000.1/examples/example1.cpp \
../armadillo-5.000.1/examples/example_lsq.cpp 

OBJS += \
./armadillo-5.000.1/examples/example1.o \
./armadillo-5.000.1/examples/example_lsq.o 

CPP_DEPS += \
./armadillo-5.000.1/examples/example1.d \
./armadillo-5.000.1/examples/example_lsq.d 


# Each subdirectory must supply rules for building sources it contributes
armadillo-5.000.1/examples/%.o: ../armadillo-5.000.1/examples/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


