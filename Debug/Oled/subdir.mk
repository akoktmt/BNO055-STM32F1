################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Oled/fonts.c \
../Oled/ssd1306.c \
../Oled/test.c 

OBJS += \
./Oled/fonts.o \
./Oled/ssd1306.o \
./Oled/test.o 

C_DEPS += \
./Oled/fonts.d \
./Oled/ssd1306.d \
./Oled/test.d 


# Each subdirectory must supply rules for building sources it contributes
Oled/%.o Oled/%.su Oled/%.cyclo: ../Oled/%.c Oled/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Oled -I../BNO055 -I../LKF -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../CAN_Handle -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Oled

clean-Oled:
	-$(RM) ./Oled/fonts.cyclo ./Oled/fonts.d ./Oled/fonts.o ./Oled/fonts.su ./Oled/ssd1306.cyclo ./Oled/ssd1306.d ./Oled/ssd1306.o ./Oled/ssd1306.su ./Oled/test.cyclo ./Oled/test.d ./Oled/test.o ./Oled/test.su

.PHONY: clean-Oled

