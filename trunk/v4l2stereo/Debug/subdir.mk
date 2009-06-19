################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../anyoption.cpp \
../drawing.cpp \
../libcam.cpp \
../main.cpp \
../polynomial.cpp \
../stereo.cpp 

OBJS += \
./anyoption.o \
./drawing.o \
./libcam.o \
./main.o \
./polynomial.o \
./stereo.o 

CPP_DEPS += \
./anyoption.d \
./drawing.d \
./libcam.d \
./main.d \
./polynomial.d \
./stereo.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/opencv/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -lcam -lcv -lcxcore -lcvaux -lhighgui -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


