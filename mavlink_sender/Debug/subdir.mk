################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../mavlinkWriter.cpp \
../mavlink_sender.cpp 

OBJS += \
./mavlinkWriter.o \
./mavlink_sender.o 

CPP_DEPS += \
./mavlinkWriter.d \
./mavlink_sender.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/Users/Maverick/Documents/workspace/Libraries/libftdi1-1.2/src -I"/Users/Maverick/Documents/workspace/mavlink_sender/include/mavlink/v1.0" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


