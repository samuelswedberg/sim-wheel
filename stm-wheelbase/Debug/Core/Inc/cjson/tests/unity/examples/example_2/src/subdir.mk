################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode.c \
../Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode2.c 

OBJS += \
./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode.o \
./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode2.o 

C_DEPS += \
./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode.d \
./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode2.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/cjson/tests/unity/examples/example_2/src/%.o Core/Inc/cjson/tests/unity/examples/example_2/src/%.su Core/Inc/cjson/tests/unity/examples/example_2/src/%.cyclo: ../Core/Inc/cjson/tests/unity/examples/example_2/src/%.c Core/Inc/cjson/tests/unity/examples/example_2/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_2-2f-src

clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_2-2f-src:
	-$(RM) ./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode.cyclo ./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode.d ./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode.o ./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode.su ./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode2.cyclo ./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode2.d ./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode2.o ./Core/Inc/cjson/tests/unity/examples/example_2/src/ProductionCode2.su

.PHONY: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_2-2f-src

