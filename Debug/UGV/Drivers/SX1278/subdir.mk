################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UGV/Drivers/SX1278/SX1278.c \
../UGV/Drivers/SX1278/SX1278_hw.c 

OBJS += \
./UGV/Drivers/SX1278/SX1278.o \
./UGV/Drivers/SX1278/SX1278_hw.o 

C_DEPS += \
./UGV/Drivers/SX1278/SX1278.d \
./UGV/Drivers/SX1278/SX1278_hw.d 


# Each subdirectory must supply rules for building sources it contributes
UGV/Drivers/SX1278/%.o UGV/Drivers/SX1278/%.su UGV/Drivers/SX1278/%.cyclo: ../UGV/Drivers/SX1278/%.c UGV/Drivers/SX1278/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F334x8 -c -I../Core/Inc -I"C:/Users/Mehmet Dincer/Desktop/Embedded systems/NRF24/001_UGV_GROUND_STATION/UGV/Drivers/crc" -I"C:/Users/Mehmet Dincer/Desktop/Embedded systems/NRF24/001_UGV_GROUND_STATION/UGV/Drivers/SX1278" -I"C:/Users/Mehmet Dincer/Desktop/Embedded systems/NRF24/001_UGV_GROUND_STATION/UGV/Main" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UGV-2f-Drivers-2f-SX1278

clean-UGV-2f-Drivers-2f-SX1278:
	-$(RM) ./UGV/Drivers/SX1278/SX1278.cyclo ./UGV/Drivers/SX1278/SX1278.d ./UGV/Drivers/SX1278/SX1278.o ./UGV/Drivers/SX1278/SX1278.su ./UGV/Drivers/SX1278/SX1278_hw.cyclo ./UGV/Drivers/SX1278/SX1278_hw.d ./UGV/Drivers/SX1278/SX1278_hw.o ./UGV/Drivers/SX1278/SX1278_hw.su

.PHONY: clean-UGV-2f-Drivers-2f-SX1278

