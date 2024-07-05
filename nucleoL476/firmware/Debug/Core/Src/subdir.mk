################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/globalvar.c \
../Core/Src/gnss_nmea_parser.c \
../Core/Src/gnss_ubx_parser.c \
../Core/Src/main.c \
../Core/Src/scheduler.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/globalvar.o \
./Core/Src/gnss_nmea_parser.o \
./Core/Src/gnss_ubx_parser.o \
./Core/Src/main.o \
./Core/Src/scheduler.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/globalvar.d \
./Core/Src/gnss_nmea_parser.d \
./Core/Src/gnss_ubx_parser.d \
./Core/Src/main.d \
./Core/Src/scheduler.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_L476RG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/BSP/can -I../Drivers/BSP/teseo_liv3f -I../Drivers/BSP/hts221 -I../Drivers/BSP/lps22hb -I../Drivers/BSP/lsm303agr -I../Drivers/BSP/lsm6dsl -I../Drivers/BSP/common -I../Drivers/AHRS -I../Core/Inc -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/globalvar.d ./Core/Src/globalvar.o ./Core/Src/globalvar.su ./Core/Src/gnss_nmea_parser.d ./Core/Src/gnss_nmea_parser.o ./Core/Src/gnss_nmea_parser.su ./Core/Src/gnss_ubx_parser.d ./Core/Src/gnss_ubx_parser.o ./Core/Src/gnss_ubx_parser.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/scheduler.d ./Core/Src/scheduler.o ./Core/Src/scheduler.su ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src

