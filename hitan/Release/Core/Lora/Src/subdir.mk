################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lora/Src/Lora.c \
../Core/Lora/Src/lora_communication.c 

C_DEPS += \
./Core/Lora/Src/Lora.d \
./Core/Lora/Src/lora_communication.d 

OBJS += \
./Core/Lora/Src/Lora.o \
./Core/Lora/Src/lora_communication.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Lora/Src/%.o Core/Lora/Src/%.su Core/Lora/Src/%.cyclo: ../Core/Lora/Src/%.c Core/Lora/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32U5A5xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32U5xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lora-2f-Src

clean-Core-2f-Lora-2f-Src:
	-$(RM) ./Core/Lora/Src/Lora.cyclo ./Core/Lora/Src/Lora.d ./Core/Lora/Src/Lora.o ./Core/Lora/Src/Lora.su ./Core/Lora/Src/lora_communication.cyclo ./Core/Lora/Src/lora_communication.d ./Core/Lora/Src/lora_communication.o ./Core/Lora/Src/lora_communication.su

.PHONY: clean-Core-2f-Lora-2f-Src

