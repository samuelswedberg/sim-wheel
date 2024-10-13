################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/cjson/cJSON.c \
../Core/Inc/cjson/cJSON_Utils.c \
../Core/Inc/cjson/test.c 

OBJS += \
./Core/Inc/cjson/cJSON.o \
./Core/Inc/cjson/cJSON_Utils.o \
./Core/Inc/cjson/test.o 

C_DEPS += \
./Core/Inc/cjson/cJSON.d \
./Core/Inc/cjson/cJSON_Utils.d \
./Core/Inc/cjson/test.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/cjson/%.o Core/Inc/cjson/%.su Core/Inc/cjson/%.cyclo: ../Core/Inc/cjson/%.c Core/Inc/cjson/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-cjson

clean-Core-2f-Inc-2f-cjson:
	-$(RM) ./Core/Inc/cjson/cJSON.cyclo ./Core/Inc/cjson/cJSON.d ./Core/Inc/cjson/cJSON.o ./Core/Inc/cjson/cJSON.su ./Core/Inc/cjson/cJSON_Utils.cyclo ./Core/Inc/cjson/cJSON_Utils.d ./Core/Inc/cjson/cJSON_Utils.o ./Core/Inc/cjson/cJSON_Utils.su ./Core/Inc/cjson/test.cyclo ./Core/Inc/cjson/test.d ./Core/Inc/cjson/test.o ./Core/Inc/cjson/test.su

.PHONY: clean-Core-2f-Inc-2f-cjson

