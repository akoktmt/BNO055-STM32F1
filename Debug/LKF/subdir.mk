################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LKF/LKF.c \
../LKF/math.c 

OBJS += \
./LKF/LKF.o \
./LKF/math.o 

C_DEPS += \
./LKF/LKF.d \
./LKF/math.d 


# Each subdirectory must supply rules for building sources it contributes
LKF/%.o LKF/%.su LKF/%.cyclo: ../LKF/%.c LKF/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Oled -I../BNO055 -I../LKF -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../CAN_Handle -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-LKF

clean-LKF:
	-$(RM) ./LKF/LKF.cyclo ./LKF/LKF.d ./LKF/LKF.o ./LKF/LKF.su ./LKF/math.cyclo ./LKF/math.d ./LKF/math.o ./LKF/math.su

.PHONY: clean-LKF

