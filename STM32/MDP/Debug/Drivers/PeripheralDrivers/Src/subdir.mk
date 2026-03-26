################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/PeripheralDrivers/Src/oled.c 

C_DEPS += \
./Drivers/PeripheralDrivers/Src/oled.d 

OBJS += \
./Drivers/PeripheralDrivers/Src/oled.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/PeripheralDrivers/Src/%.o Drivers/PeripheralDrivers/Src/%.su Drivers/PeripheralDrivers/Src/%.cyclo: ../Drivers/PeripheralDrivers/Src/%.c Drivers/PeripheralDrivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/LiYujia/SC2079-MDP-Group-24-main/STM32/MDP/Drivers/PeripheralDrivers/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-PeripheralDrivers-2f-Src

clean-Drivers-2f-PeripheralDrivers-2f-Src:
	-$(RM) ./Drivers/PeripheralDrivers/Src/oled.cyclo ./Drivers/PeripheralDrivers/Src/oled.d ./Drivers/PeripheralDrivers/Src/oled.o ./Drivers/PeripheralDrivers/Src/oled.su

.PHONY: clean-Drivers-2f-PeripheralDrivers-2f-Src

