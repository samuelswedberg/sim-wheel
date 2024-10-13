################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/cjson/fuzzing/afl.c \
../Core/Inc/cjson/fuzzing/cjson_read_fuzzer.c \
../Core/Inc/cjson/fuzzing/fuzz_main.c 

OBJS += \
./Core/Inc/cjson/fuzzing/afl.o \
./Core/Inc/cjson/fuzzing/cjson_read_fuzzer.o \
./Core/Inc/cjson/fuzzing/fuzz_main.o 

C_DEPS += \
./Core/Inc/cjson/fuzzing/afl.d \
./Core/Inc/cjson/fuzzing/cjson_read_fuzzer.d \
./Core/Inc/cjson/fuzzing/fuzz_main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/cjson/fuzzing/%.o Core/Inc/cjson/fuzzing/%.su Core/Inc/cjson/fuzzing/%.cyclo: ../Core/Inc/cjson/fuzzing/%.c Core/Inc/cjson/fuzzing/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-cjson-2f-fuzzing

clean-Core-2f-Inc-2f-cjson-2f-fuzzing:
	-$(RM) ./Core/Inc/cjson/fuzzing/afl.cyclo ./Core/Inc/cjson/fuzzing/afl.d ./Core/Inc/cjson/fuzzing/afl.o ./Core/Inc/cjson/fuzzing/afl.su ./Core/Inc/cjson/fuzzing/cjson_read_fuzzer.cyclo ./Core/Inc/cjson/fuzzing/cjson_read_fuzzer.d ./Core/Inc/cjson/fuzzing/cjson_read_fuzzer.o ./Core/Inc/cjson/fuzzing/cjson_read_fuzzer.su ./Core/Inc/cjson/fuzzing/fuzz_main.cyclo ./Core/Inc/cjson/fuzzing/fuzz_main.d ./Core/Inc/cjson/fuzzing/fuzz_main.o ./Core/Inc/cjson/fuzzing/fuzz_main.su

.PHONY: clean-Core-2f-Inc-2f-cjson-2f-fuzzing

