################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/cjson/tests/cjson_add.c \
../Core/Inc/cjson/tests/compare_tests.c \
../Core/Inc/cjson/tests/json_patch_tests.c \
../Core/Inc/cjson/tests/minify_tests.c \
../Core/Inc/cjson/tests/misc_tests.c \
../Core/Inc/cjson/tests/misc_utils_tests.c \
../Core/Inc/cjson/tests/old_utils_tests.c \
../Core/Inc/cjson/tests/parse_array.c \
../Core/Inc/cjson/tests/parse_examples.c \
../Core/Inc/cjson/tests/parse_hex4.c \
../Core/Inc/cjson/tests/parse_number.c \
../Core/Inc/cjson/tests/parse_object.c \
../Core/Inc/cjson/tests/parse_string.c \
../Core/Inc/cjson/tests/parse_value.c \
../Core/Inc/cjson/tests/parse_with_opts.c \
../Core/Inc/cjson/tests/print_array.c \
../Core/Inc/cjson/tests/print_number.c \
../Core/Inc/cjson/tests/print_object.c \
../Core/Inc/cjson/tests/print_string.c \
../Core/Inc/cjson/tests/print_value.c \
../Core/Inc/cjson/tests/readme_examples.c \
../Core/Inc/cjson/tests/unity_setup.c 

OBJS += \
./Core/Inc/cjson/tests/cjson_add.o \
./Core/Inc/cjson/tests/compare_tests.o \
./Core/Inc/cjson/tests/json_patch_tests.o \
./Core/Inc/cjson/tests/minify_tests.o \
./Core/Inc/cjson/tests/misc_tests.o \
./Core/Inc/cjson/tests/misc_utils_tests.o \
./Core/Inc/cjson/tests/old_utils_tests.o \
./Core/Inc/cjson/tests/parse_array.o \
./Core/Inc/cjson/tests/parse_examples.o \
./Core/Inc/cjson/tests/parse_hex4.o \
./Core/Inc/cjson/tests/parse_number.o \
./Core/Inc/cjson/tests/parse_object.o \
./Core/Inc/cjson/tests/parse_string.o \
./Core/Inc/cjson/tests/parse_value.o \
./Core/Inc/cjson/tests/parse_with_opts.o \
./Core/Inc/cjson/tests/print_array.o \
./Core/Inc/cjson/tests/print_number.o \
./Core/Inc/cjson/tests/print_object.o \
./Core/Inc/cjson/tests/print_string.o \
./Core/Inc/cjson/tests/print_value.o \
./Core/Inc/cjson/tests/readme_examples.o \
./Core/Inc/cjson/tests/unity_setup.o 

C_DEPS += \
./Core/Inc/cjson/tests/cjson_add.d \
./Core/Inc/cjson/tests/compare_tests.d \
./Core/Inc/cjson/tests/json_patch_tests.d \
./Core/Inc/cjson/tests/minify_tests.d \
./Core/Inc/cjson/tests/misc_tests.d \
./Core/Inc/cjson/tests/misc_utils_tests.d \
./Core/Inc/cjson/tests/old_utils_tests.d \
./Core/Inc/cjson/tests/parse_array.d \
./Core/Inc/cjson/tests/parse_examples.d \
./Core/Inc/cjson/tests/parse_hex4.d \
./Core/Inc/cjson/tests/parse_number.d \
./Core/Inc/cjson/tests/parse_object.d \
./Core/Inc/cjson/tests/parse_string.d \
./Core/Inc/cjson/tests/parse_value.d \
./Core/Inc/cjson/tests/parse_with_opts.d \
./Core/Inc/cjson/tests/print_array.d \
./Core/Inc/cjson/tests/print_number.d \
./Core/Inc/cjson/tests/print_object.d \
./Core/Inc/cjson/tests/print_string.d \
./Core/Inc/cjson/tests/print_value.d \
./Core/Inc/cjson/tests/readme_examples.d \
./Core/Inc/cjson/tests/unity_setup.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/cjson/tests/%.o Core/Inc/cjson/tests/%.su Core/Inc/cjson/tests/%.cyclo: ../Core/Inc/cjson/tests/%.c Core/Inc/cjson/tests/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-cjson-2f-tests

clean-Core-2f-Inc-2f-cjson-2f-tests:
	-$(RM) ./Core/Inc/cjson/tests/cjson_add.cyclo ./Core/Inc/cjson/tests/cjson_add.d ./Core/Inc/cjson/tests/cjson_add.o ./Core/Inc/cjson/tests/cjson_add.su ./Core/Inc/cjson/tests/compare_tests.cyclo ./Core/Inc/cjson/tests/compare_tests.d ./Core/Inc/cjson/tests/compare_tests.o ./Core/Inc/cjson/tests/compare_tests.su ./Core/Inc/cjson/tests/json_patch_tests.cyclo ./Core/Inc/cjson/tests/json_patch_tests.d ./Core/Inc/cjson/tests/json_patch_tests.o ./Core/Inc/cjson/tests/json_patch_tests.su ./Core/Inc/cjson/tests/minify_tests.cyclo ./Core/Inc/cjson/tests/minify_tests.d ./Core/Inc/cjson/tests/minify_tests.o ./Core/Inc/cjson/tests/minify_tests.su ./Core/Inc/cjson/tests/misc_tests.cyclo ./Core/Inc/cjson/tests/misc_tests.d ./Core/Inc/cjson/tests/misc_tests.o ./Core/Inc/cjson/tests/misc_tests.su ./Core/Inc/cjson/tests/misc_utils_tests.cyclo ./Core/Inc/cjson/tests/misc_utils_tests.d ./Core/Inc/cjson/tests/misc_utils_tests.o ./Core/Inc/cjson/tests/misc_utils_tests.su ./Core/Inc/cjson/tests/old_utils_tests.cyclo ./Core/Inc/cjson/tests/old_utils_tests.d ./Core/Inc/cjson/tests/old_utils_tests.o ./Core/Inc/cjson/tests/old_utils_tests.su ./Core/Inc/cjson/tests/parse_array.cyclo ./Core/Inc/cjson/tests/parse_array.d ./Core/Inc/cjson/tests/parse_array.o ./Core/Inc/cjson/tests/parse_array.su ./Core/Inc/cjson/tests/parse_examples.cyclo ./Core/Inc/cjson/tests/parse_examples.d ./Core/Inc/cjson/tests/parse_examples.o ./Core/Inc/cjson/tests/parse_examples.su ./Core/Inc/cjson/tests/parse_hex4.cyclo ./Core/Inc/cjson/tests/parse_hex4.d ./Core/Inc/cjson/tests/parse_hex4.o ./Core/Inc/cjson/tests/parse_hex4.su ./Core/Inc/cjson/tests/parse_number.cyclo ./Core/Inc/cjson/tests/parse_number.d ./Core/Inc/cjson/tests/parse_number.o ./Core/Inc/cjson/tests/parse_number.su ./Core/Inc/cjson/tests/parse_object.cyclo ./Core/Inc/cjson/tests/parse_object.d ./Core/Inc/cjson/tests/parse_object.o ./Core/Inc/cjson/tests/parse_object.su ./Core/Inc/cjson/tests/parse_string.cyclo ./Core/Inc/cjson/tests/parse_string.d ./Core/Inc/cjson/tests/parse_string.o ./Core/Inc/cjson/tests/parse_string.su ./Core/Inc/cjson/tests/parse_value.cyclo ./Core/Inc/cjson/tests/parse_value.d ./Core/Inc/cjson/tests/parse_value.o ./Core/Inc/cjson/tests/parse_value.su ./Core/Inc/cjson/tests/parse_with_opts.cyclo ./Core/Inc/cjson/tests/parse_with_opts.d ./Core/Inc/cjson/tests/parse_with_opts.o ./Core/Inc/cjson/tests/parse_with_opts.su ./Core/Inc/cjson/tests/print_array.cyclo ./Core/Inc/cjson/tests/print_array.d ./Core/Inc/cjson/tests/print_array.o ./Core/Inc/cjson/tests/print_array.su ./Core/Inc/cjson/tests/print_number.cyclo ./Core/Inc/cjson/tests/print_number.d ./Core/Inc/cjson/tests/print_number.o ./Core/Inc/cjson/tests/print_number.su ./Core/Inc/cjson/tests/print_object.cyclo ./Core/Inc/cjson/tests/print_object.d ./Core/Inc/cjson/tests/print_object.o ./Core/Inc/cjson/tests/print_object.su ./Core/Inc/cjson/tests/print_string.cyclo ./Core/Inc/cjson/tests/print_string.d ./Core/Inc/cjson/tests/print_string.o ./Core/Inc/cjson/tests/print_string.su ./Core/Inc/cjson/tests/print_value.cyclo ./Core/Inc/cjson/tests/print_value.d ./Core/Inc/cjson/tests/print_value.o ./Core/Inc/cjson/tests/print_value.su ./Core/Inc/cjson/tests/readme_examples.cyclo ./Core/Inc/cjson/tests/readme_examples.d ./Core/Inc/cjson/tests/readme_examples.o ./Core/Inc/cjson/tests/readme_examples.su ./Core/Inc/cjson/tests/unity_setup.cyclo ./Core/Inc/cjson/tests/unity_setup.d ./Core/Inc/cjson/tests/unity_setup.o ./Core/Inc/cjson/tests/unity_setup.su

.PHONY: clean-Core-2f-Inc-2f-cjson-2f-tests

