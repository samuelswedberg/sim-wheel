################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/cjson/tests/unity/extras/fixture/test/template_fixture_tests.c \
../Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_Test.c \
../Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_TestRunner.c \
../Core/Inc/cjson/tests/unity/extras/fixture/test/unity_output_Spy.c 

OBJS += \
./Core/Inc/cjson/tests/unity/extras/fixture/test/template_fixture_tests.o \
./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_Test.o \
./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_TestRunner.o \
./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_output_Spy.o 

C_DEPS += \
./Core/Inc/cjson/tests/unity/extras/fixture/test/template_fixture_tests.d \
./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_Test.d \
./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_TestRunner.d \
./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_output_Spy.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/cjson/tests/unity/extras/fixture/test/%.o Core/Inc/cjson/tests/unity/extras/fixture/test/%.su Core/Inc/cjson/tests/unity/extras/fixture/test/%.cyclo: ../Core/Inc/cjson/tests/unity/extras/fixture/test/%.c Core/Inc/cjson/tests/unity/extras/fixture/test/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-extras-2f-fixture-2f-test

clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-extras-2f-fixture-2f-test:
	-$(RM) ./Core/Inc/cjson/tests/unity/extras/fixture/test/template_fixture_tests.cyclo ./Core/Inc/cjson/tests/unity/extras/fixture/test/template_fixture_tests.d ./Core/Inc/cjson/tests/unity/extras/fixture/test/template_fixture_tests.o ./Core/Inc/cjson/tests/unity/extras/fixture/test/template_fixture_tests.su ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_Test.cyclo ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_Test.d ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_Test.o ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_Test.su ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_TestRunner.cyclo ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_TestRunner.d ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_TestRunner.o ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_fixture_TestRunner.su ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_output_Spy.cyclo ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_output_Spy.d ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_output_Spy.o ./Core/Inc/cjson/tests/unity/extras/fixture/test/unity_output_Spy.su

.PHONY: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-extras-2f-fixture-2f-test

