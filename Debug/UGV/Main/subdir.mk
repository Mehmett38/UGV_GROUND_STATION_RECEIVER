################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UGV/Main/ugvMain.c 

OBJS += \
./UGV/Main/ugvMain.o 

C_DEPS += \
./UGV/Main/ugvMain.d 


# Each subdirectory must supply rules for building sources it contributes
UGV/Main/%.o UGV/Main/%.su UGV/Main/%.cyclo: ../UGV/Main/%.c UGV/Main/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F334x8 -c -I../Core/Inc -I"C:/Users/Mehmet Dincer/Desktop/Embedded systems/NRF24/001_UGV_GROUND_STATION/UGV/Drivers/crc" -I"C:/Users/Mehmet Dincer/Desktop/Embedded systems/NRF24/001_UGV_GROUND_STATION/UGV/Drivers/SX1278" -I"C:/Users/Mehmet Dincer/Desktop/Embedded systems/NRF24/001_UGV_GROUND_STATION/UGV/Main" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UGV-2f-Main

clean-UGV-2f-Main:
	-$(RM) ./UGV/Main/ugvMain.cyclo ./UGV/Main/ugvMain.d ./UGV/Main/ugvMain.o ./UGV/Main/ugvMain.su

.PHONY: clean-UGV-2f-Main

