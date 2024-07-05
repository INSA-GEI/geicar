################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/globalvar.c \
../Application/imu.c \
../Application/scheduler.c \
../Application/trames_nmea.c 

OBJS += \
./Application/globalvar.o \
./Application/imu.o \
./Application/scheduler.o \
./Application/trames_nmea.o 

C_DEPS += \
./Application/globalvar.d \
./Application/imu.d \
./Application/scheduler.d \
./Application/trames_nmea.d 


# Each subdirectory must supply rules for building sources it contributes
Application/%.o Application/%.su Application/%.cyclo: ../Application/%.c Application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/BSP/Components/lsm6dsl -I../Drivers/BSP/Components/hts221 -I../Drivers/BSP/Components/lps22hb -I../Drivers/BSP/Components/lsm303agr -I../X-CUBE-MEMS1/Target -I"C:/Users/hamd/STM32CubeIDE/workspace_1.12.1/L476_project/Application" -I../Application -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application

clean-Application:
	-$(RM) ./Application/globalvar.cyclo ./Application/globalvar.d ./Application/globalvar.o ./Application/globalvar.su ./Application/imu.cyclo ./Application/imu.d ./Application/imu.o ./Application/imu.su ./Application/scheduler.cyclo ./Application/scheduler.d ./Application/scheduler.o ./Application/scheduler.su ./Application/trames_nmea.cyclo ./Application/trames_nmea.d ./Application/trames_nmea.o ./Application/trames_nmea.su

.PHONY: clean-Application

