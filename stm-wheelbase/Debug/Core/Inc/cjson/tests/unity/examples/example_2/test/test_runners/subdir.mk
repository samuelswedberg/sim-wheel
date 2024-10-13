################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode2_Runner.c \
../Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode_Runner.c \
../Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/all_tests.c 

OBJS += \
./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode2_Runner.o \
./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode_Runner.o \
./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/all_tests.o 

C_DEPS += \
./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode2_Runner.d \
./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode_Runner.d \
./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/all_tests.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/%.o Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/%.su Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/%.cyclo: ../Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/%.c Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_2-2f-test-2f-test_runners

clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_2-2f-test-2f-test_runners:
	-$(RM) ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode2_Runner.cyclo ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode2_Runner.d ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode2_Runner.o ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode2_Runner.su ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode_Runner.cyclo ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode_Runner.d ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode_Runner.o ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/TestProductionCode_Runner.su ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/all_tests.cyclo ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/all_tests.d ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/all_tests.o ./Core/Inc/cjson/tests/unity/examples/example_2/test/test_runners/all_tests.su

.PHONY: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-examples-2f-example_2-2f-test-2f-test_runners

