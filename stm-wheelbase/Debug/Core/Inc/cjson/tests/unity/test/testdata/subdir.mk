################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/cjson/tests/unity/test/testdata/testRunnerGenerator.c \
../Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorSmall.c \
../Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorWithMocks.c 

OBJS += \
./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGenerator.o \
./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorSmall.o \
./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorWithMocks.o 

C_DEPS += \
./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGenerator.d \
./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorSmall.d \
./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorWithMocks.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/cjson/tests/unity/test/testdata/%.o Core/Inc/cjson/tests/unity/test/testdata/%.su Core/Inc/cjson/tests/unity/test/testdata/%.cyclo: ../Core/Inc/cjson/tests/unity/test/testdata/%.c Core/Inc/cjson/tests/unity/test/testdata/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-test-2f-testdata

clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-test-2f-testdata:
	-$(RM) ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGenerator.cyclo ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGenerator.d ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGenerator.o ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGenerator.su ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorSmall.cyclo ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorSmall.d ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorSmall.o ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorSmall.su ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorWithMocks.cyclo ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorWithMocks.d ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorWithMocks.o ./Core/Inc/cjson/tests/unity/test/testdata/testRunnerGeneratorWithMocks.su

.PHONY: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-test-2f-testdata

