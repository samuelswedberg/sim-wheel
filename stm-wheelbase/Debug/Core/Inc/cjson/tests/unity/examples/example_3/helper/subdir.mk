################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/cjson/tests/unity/examples/example_3/helper/UnityHelper.c 

OBJS += \
./Core/Inc/cjson/tests/unity/examples/example_3/helper/UnityHelper.o 

C_DEPS += \
./Core/Inc/cjson/tests/unity/examples/example_3/helper/UnityHelper.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/cjson/tests/unity/examples/example_3/helper/%.o Core/Inc/cjson/tests/unity/examples/example_3/helper/%.su Core/Inc/cjson/tests/unity/examples/example_3/helper/%.cyclo: ../Core/Inc/cjson/tests/unity/examples/example_3/helper/%.c Core/Inc/cjson/tests/unity/examples/example_3/helper/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_3-2f-helper

clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_3-2f-helper:
	-$(RM) ./Core/Inc/cjson/tests/unity/examples/example_3/helper/UnityHelper.cyclo ./Core/Inc/cjson/tests/unity/examples/example_3/helper/UnityHelper.d ./Core/Inc/cjson/tests/unity/examples/example_3/helper/UnityHelper.o ./Core/Inc/cjson/tests/unity/examples/example_3/helper/UnityHelper.su

.PHONY: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_3-2f-helper

