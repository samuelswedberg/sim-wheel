################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode.c \
../Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode2.c 

OBJS += \
./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode.o \
./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode2.o 

C_DEPS += \
./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode.d \
./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode2.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/cjson/tests/unity/examples/example_1/test/%.o Core/Inc/cjson/tests/unity/examples/example_1/test/%.su Core/Inc/cjson/tests/unity/examples/example_1/test/%.cyclo: ../Core/Inc/cjson/tests/unity/examples/example_1/test/%.c Core/Inc/cjson/tests/unity/examples/example_1/test/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_1-2f-test

clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_1-2f-test:
	-$(RM) ./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode.cyclo ./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode.d ./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode.o ./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode.su ./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode2.cyclo ./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode2.d ./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode2.o ./Core/Inc/cjson/tests/unity/examples/example_1/test/TestProductionCode2.su

.PHONY: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_1-2f-test
