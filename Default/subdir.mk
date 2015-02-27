################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Epos.cpp \
../EposComm.cpp \
../TAGPSTracking.cpp \
../TAGUIBackend.cpp \
../TALogger.cpp \
../TAMotorControl.cpp \
../TARecorder.cpp \
../TATrackingMode.cpp \
../TAinput.cpp \
../TAmode.cpp \
../TSPosInput.cpp \
../TSfindNorth.cpp \
../TSmavlinkGPS.cpp \
../TSmavlinkMag.cpp \
../TSmavlinkRadioStatus.cpp \
../TSmavlinkReader.cpp \
../config.cpp \
../trackingsetup.cpp 

OBJS += \
./Epos.o \
./EposComm.o \
./TAGPSTracking.o \
./TAGUIBackend.o \
./TALogger.o \
./TAMotorControl.o \
./TARecorder.o \
./TATrackingMode.o \
./TAinput.o \
./TAmode.o \
./TSPosInput.o \
./TSfindNorth.o \
./TSmavlinkGPS.o \
./TSmavlinkMag.o \
./TSmavlinkRadioStatus.o \
./TSmavlinkReader.o \
./config.o \
./trackingsetup.o 

CPP_DEPS += \
./Epos.d \
./EposComm.d \
./TAGPSTracking.d \
./TAGUIBackend.d \
./TALogger.d \
./TAMotorControl.d \
./TARecorder.d \
./TATrackingMode.d \
./TAinput.d \
./TAmode.d \
./TSPosInput.d \
./TSfindNorth.d \
./TSmavlinkGPS.d \
./TSmavlinkMag.d \
./TSmavlinkRadioStatus.d \
./TSmavlinkReader.d \
./config.d \
./trackingsetup.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/asl/workspace/TrackingSetup/includes/mavlink/v1.0" -I"/home/asl/workspace/TrackingSetup/includes" -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


