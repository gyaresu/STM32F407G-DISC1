################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/BufferPool.c \
../Src/car_control.c \
../Src/cc1101.c \
../Src/dma.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/spi.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c \
../Src/usart.c 

OBJS += \
./Src/BufferPool.o \
./Src/car_control.o \
./Src/cc1101.o \
./Src/dma.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/spi.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o \
./Src/usart.o 

C_DEPS += \
./Src/BufferPool.d \
./Src/car_control.d \
./Src/cc1101.d \
./Src/dma.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/spi.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F407xx -I"/Users/gyaresu/programming/stm32-discovery/spi_test/Inc" -I"/Users/gyaresu/programming/stm32-discovery/spi_test/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/Users/gyaresu/programming/stm32-discovery/spi_test/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/Users/gyaresu/programming/stm32-discovery/spi_test/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/gyaresu/programming/stm32-discovery/spi_test/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


