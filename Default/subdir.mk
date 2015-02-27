################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Epos.cpp \
../EposComm.cpp \
../TAClass.cpp \
../TAConfig.cpp \
../TAGPSTracking.cpp \
../TAGUIBackend.cpp \
../TALogger.cpp \
../TAMappingEstimation.cpp \
../TAMotorControl.cpp \
../TARecorder.cpp \
../TATrackingMode.cpp \
../TAinput.cpp \
../TAmode.cpp \
../TSPosInput.cpp \
../TSfindNorth.cpp \
../TSmagneticDeclination.cpp \
../TSmavlinkGPS.cpp \
../TSmavlinkMag.cpp \
../TSmavlinkReader.cpp \
../TSmavlinkRadioStatus.cpp \
../TrackingSetup.cpp 

OBJS += \
./Epos.o \
./EposComm.o \
./TAClass.o \
./TAConfig.o \
./TAGPSTracking.o \
./TAGUIBackend.o \
./TALogger.o \
./TAMappingEstimation.o \
./TAMotorControl.o \
./TARecorder.o \
./TATrackingMode.o \
./TAinput.o \
./TAmode.o \
./TSPosInput.o \
./TSfindNorth.o \
./TSmagneticDeclination.o \
./TSmavlinkGPS.o \
./TSmavlinkMag.o \
./TSmavlinkReader.o \
./TSmavlinkRadioStatus.o \
./TrackingSetup.o 

CPP_DEPS += \
./Epos.d \
./EposComm.d \
./TAClass.d \
./TAConfig.d \
./TAGPSTracking.d \
./TAGUIBackend.d \
./TALogger.d \
./TAMappingEstimation.d \
./TAMotorControl.d \
./TARecorder.d \
./TATrackingMode.d \
./TAinput.d \
./TAmode.d \
./TSPosInput.d \
./TSfindNorth.d \
./TSmagneticDeclination.d \
./TSmavlinkGPS.d \
./TSmavlinkMag.d \
./TSmavlinkReader.d \
./TSmavlinkRadioStatus.d \
./TrackingSetup.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/odroid/TrackingSetup/mavlink/include/mavlink/v1.0" -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


