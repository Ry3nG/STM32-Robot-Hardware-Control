################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/encoder_task.c \
../App/Src/led_task.c \
../App/Src/motor_task.c \
../App/Src/oled_task.c \
../App/Src/pid.c \
../App/Src/servo_task.c \
../App/Src/ultrasonic_task.c 

OBJS += \
./App/Src/encoder_task.o \
./App/Src/led_task.o \
./App/Src/motor_task.o \
./App/Src/oled_task.o \
./App/Src/pid.o \
./App/Src/servo_task.o \
./App/Src/ultrasonic_task.o 

C_DEPS += \
./App/Src/encoder_task.d \
./App/Src/led_task.d \
./App/Src/motor_task.d \
./App/Src/oled_task.d \
./App/Src/pid.d \
./App/Src/servo_task.d \
./App/Src/ultrasonic_task.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../App/Inc -I"D:/Coding/MDP-Group1/MDP_V1/PeripheralDriver/Inc" -I"D:/Coding/MDP-Group1/MDP_V1/PeripheralDriver/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
App/Src/motor_task.o: ../App/Src/motor_task.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../App/Inc -I"D:/Coding/MDP-Group1/MDP_V1/PeripheralDriver/Inc" -I"D:/Coding/MDP-Group1/MDP_V1/PeripheralDriver/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
App/Src/ultrasonic_task.o: ../App/Src/ultrasonic_task.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../App/Inc -I"D:/Coding/MDP-Group1/MDP_V1/PeripheralDriver/Inc" -I"D:/Coding/MDP-Group1/MDP_V1/PeripheralDriver/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/encoder_task.d ./App/Src/encoder_task.o ./App/Src/encoder_task.su ./App/Src/led_task.d ./App/Src/led_task.o ./App/Src/led_task.su ./App/Src/motor_task.d ./App/Src/motor_task.o ./App/Src/motor_task.su ./App/Src/oled_task.d ./App/Src/oled_task.o ./App/Src/oled_task.su ./App/Src/pid.d ./App/Src/pid.o ./App/Src/pid.su ./App/Src/servo_task.d ./App/Src/servo_task.o ./App/Src/servo_task.su ./App/Src/ultrasonic_task.d ./App/Src/ultrasonic_task.o ./App/Src/ultrasonic_task.su

.PHONY: clean-App-2f-Src

