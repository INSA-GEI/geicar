################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm6dsl/lsm6dsl.c \
../Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.c 

OBJS += \
./Drivers/BSP/Components/lsm6dsl/lsm6dsl.o \
./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lsm6dsl/lsm6dsl.d \
./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm6dsl/%.o Drivers/BSP/Components/lsm6dsl/%.su Drivers/BSP/Components/lsm6dsl/%.cyclo: ../Drivers/BSP/Components/lsm6dsl/%.c Drivers/BSP/Components/lsm6dsl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/BSP/Components/lsm6dsl -I../Drivers/BSP/Components/hts221 -I../Drivers/BSP/Components/lps22hb -I../Drivers/BSP/Components/lsm303agr -I../X-CUBE-MEMS1/Target -I"U:/Windows/Bureau/STAGE_4A/git/geicar/L476_project/Application" -I/L476_project/Application -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsl

clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsl:
	-$(RM) ./Drivers/BSP/Components/lsm6dsl/lsm6dsl.cyclo ./Drivers/BSP/Components/lsm6dsl/lsm6dsl.d ./Drivers/BSP/Components/lsm6dsl/lsm6dsl.o ./Drivers/BSP/Components/lsm6dsl/lsm6dsl.su ./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.cyclo ./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.d ./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.o ./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsl

