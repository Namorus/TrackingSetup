################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../config.cpp \
../current_utc_time.cpp \
../epos.cpp \
../epos_comm.cpp \
../estimator_kf.cpp \
../find_north.cpp \
../gps_tracking.cpp \
../gui_backend.cpp \
../input.cpp \
../logger.cpp \
../mavlink_attitude.cpp \
../mavlink_gpos.cpp \
../mavlink_gps.cpp \
../mavlink_mag.cpp \
../mavlink_radio_status.cpp \
../mavlink_reader.cpp \
../mavlink_vfr_hud.cpp \
../motor_control.cpp \
../position_input.cpp \
../recorder.cpp \
../trackingEstimator.cpp \
../tracking_mode.cpp \
../tracking_state.cpp \
../trackingsetup.cpp 

OBJS += \
./config.o \
./current_utc_time.o \
./epos.o \
./epos_comm.o \
./estimator_kf.o \
./find_north.o \
./gps_tracking.o \
./gui_backend.o \
./input.o \
./logger.o \
./mavlink_attitude.o \
./mavlink_gpos.o \
./mavlink_gps.o \
./mavlink_mag.o \
./mavlink_radio_status.o \
./mavlink_reader.o \
./mavlink_vfr_hud.o \
./motor_control.o \
./position_input.o \
./recorder.o \
./trackingEstimator.o \
./tracking_mode.o \
./tracking_state.o \
./trackingsetup.o 

CPP_DEPS += \
./config.d \
./current_utc_time.d \
./epos.d \
./epos_comm.d \
./estimator_kf.d \
./find_north.d \
./gps_tracking.d \
./gui_backend.d \
./input.d \
./logger.d \
./mavlink_attitude.d \
./mavlink_gpos.d \
./mavlink_gps.d \
./mavlink_mag.d \
./mavlink_radio_status.d \
./mavlink_reader.d \
./mavlink_vfr_hud.d \
./motor_control.d \
./position_input.d \
./recorder.d \
./trackingEstimator.d \
./tracking_mode.d \
./tracking_state.d \
./trackingsetup.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I"/Users/Maverick/Documents/workspace/TS_from_working_attitude/includes/mavlink/v1.0" -I"/Users/Maverick/Documents/workspace/TS_from_working_attitude/includes" -I/Users/Maverick/Documents/workspace/Libraries/libftdi1-1.2/ftdipp -I/Users/Maverick/Documents/workspace/Libraries/libftdi1-1.2/src -I/Users/Maverick/Documents/workspace/Libraries/armadillo/include/armadillo_bits -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


