################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/lps22hb/lps22hb_reg.c 

OBJS += \
./Drivers/BSP/lps22hb/lps22hb_reg.o 

C_DEPS += \
./Drivers/BSP/lps22hb/lps22hb_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/lps22hb/%.o Drivers/BSP/lps22hb/%.su: ../Drivers/BSP/lps22hb/%.c Drivers/BSP/lps22hb/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_L476RG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/BSP/can -I../Drivers/BSP/teseo_liv3f -I../Drivers/BSP/hts221 -I../Drivers/BSP/lps22hb -I../Drivers/BSP/lsm303agr -I../Drivers/BSP/lsm6dsl -I../Drivers/BSP/common -I../Drivers/AHRS -I../Core/Inc -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-lps22hb

clean-Drivers-2f-BSP-2f-lps22hb:
	-$(RM) ./Drivers/BSP/lps22hb/lps22hb_reg.d ./Drivers/BSP/lps22hb/lps22hb_reg.o ./Drivers/BSP/lps22hb/lps22hb_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-lps22hb

