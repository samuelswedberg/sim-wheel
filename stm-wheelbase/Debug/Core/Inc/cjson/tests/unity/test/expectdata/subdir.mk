################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_cmd.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_def.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_head1.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_cmd.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_def.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_head1.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new1.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new2.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_param.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run1.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run2.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_yaml.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_new1.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_new2.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_param.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_run1.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_run2.c \
../Core/Inc/cjson/tests/unity/test/expectdata/testsample_yaml.c 

OBJS += \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_cmd.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_def.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_head1.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_cmd.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_def.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_head1.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new1.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new2.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_param.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run1.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run2.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_yaml.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new1.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new2.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_param.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run1.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run2.o \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_yaml.o 

C_DEPS += \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_cmd.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_def.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_head1.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_cmd.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_def.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_head1.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new1.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new2.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_param.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run1.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run2.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_yaml.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new1.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new2.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_param.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run1.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run2.d \
./Core/Inc/cjson/tests/unity/test/expectdata/testsample_yaml.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/cjson/tests/unity/test/expectdata/%.o Core/Inc/cjson/tests/unity/test/expectdata/%.su Core/Inc/cjson/tests/unity/test/expectdata/%.cyclo: ../Core/Inc/cjson/tests/unity/test/expectdata/%.c Core/Inc/cjson/tests/unity/test/expectdata/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-test-2f-expectdata

clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-test-2f-expectdata:
	-$(RM) ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_cmd.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_cmd.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_cmd.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_cmd.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_def.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_def.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_def.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_def.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_head1.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_head1.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_head1.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_head1.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_cmd.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_cmd.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_cmd.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_cmd.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_def.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_def.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_def.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_def.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_head1.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_head1.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_head1.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_head1.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new1.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new1.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new1.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new1.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new2.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new2.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new2.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_new2.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_param.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_param.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_param.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_param.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run1.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run1.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run1.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run1.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run2.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run2.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run2.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_run2.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_yaml.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_yaml.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_yaml.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_mock_yaml.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new1.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new1.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new1.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new1.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new2.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new2.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new2.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_new2.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_param.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_param.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_param.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_param.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run1.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run1.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run1.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run1.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run2.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run2.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run2.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_run2.su ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_yaml.cyclo ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_yaml.d ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_yaml.o ./Core/Inc/cjson/tests/unity/test/expectdata/testsample_yaml.su

.PHONY: clean-Core-2f-Inc-2f-cjson-2f-tests-2f-unity-2f-test-2f-expectdata

