################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../config.cpp \
../epos.cpp \
../epos_comm.cpp \
../find_north.cpp \
../gps_tracking.cpp \
../gui_backend.cpp \
../input.cpp \
../logger.cpp \
../mavlink_gpos.cpp \
../mavlink_gps.cpp \
../mavlink_mag.cpp \
../mavlink_radio_status.cpp \
../mavlink_reader.cpp \
../motor_control.cpp \
../position_input.cpp \
../recorder.cpp \
../tracking_mode.cpp \
../tracking_state.cpp \
../trackingsetup.cpp 

OBJS += \
./config.o \
./epos.o \
./epos_comm.o \
./find_north.o \
./gps_tracking.o \
./gui_backend.o \
./input.o \
./logger.o \
./mavlink_gpos.o \
./mavlink_gps.o \
./mavlink_mag.o \
./mavlink_radio_status.o \
./mavlink_reader.o \
./motor_control.o \
./position_input.o \
./recorder.o \
./tracking_mode.o \
./tracking_state.o \
./trackingsetup.o 

CPP_DEPS += \
./config.d \
./epos.d \
./epos_comm.d \
./find_north.d \
./gps_tracking.d \
./gui_backend.d \
./input.d \
./logger.d \
./mavlink_gpos.d \
./mavlink_gps.d \
./mavlink_mag.d \
./mavlink_radio_status.d \
./mavlink_reader.d \
./motor_control.d \
./position_input.d \
./recorder.d \
./tracking_mode.d \
./tracking_state.d \
./trackingsetup.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/asl/workspace/TrackingSetup/includes/mavlink/v1.0" -I"/home/asl/workspace/TrackingSetup/includes" -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


