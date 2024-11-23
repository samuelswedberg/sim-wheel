################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/usb/usbd_conf.c \
../Core/Src/usb/usbd_core.c \
../Core/Src/usb/usbd_ctlreq.c \
../Core/Src/usb/usbd_ioreq.c 

OBJS += \
./Core/Src/usb/usbd_conf.o \
./Core/Src/usb/usbd_core.o \
./Core/Src/usb/usbd_ctlreq.o \
./Core/Src/usb/usbd_ioreq.o 

C_DEPS += \
./Core/Src/usb/usbd_conf.d \
./Core/Src/usb/usbd_core.d \
./Core/Src/usb/usbd_ctlreq.d \
./Core/Src/usb/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/usb/%.o Core/Src/usb/%.su Core/Src/usb/%.cyclo: ../Core/Src/usb/%.c Core/Src/usb/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/Inc/usb -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Composite -I../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/COMPOSITE/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-usb

clean-Core-2f-Src-2f-usb:
	-$(RM) ./Core/Src/usb/usbd_conf.cyclo ./Core/Src/usb/usbd_conf.d ./Core/Src/usb/usbd_conf.o ./Core/Src/usb/usbd_conf.su ./Core/Src/usb/usbd_core.cyclo ./Core/Src/usb/usbd_core.d ./Core/Src/usb/usbd_core.o ./Core/Src/usb/usbd_core.su ./Core/Src/usb/usbd_ctlreq.cyclo ./Core/Src/usb/usbd_ctlreq.d ./Core/Src/usb/usbd_ctlreq.o ./Core/Src/usb/usbd_ctlreq.su ./Core/Src/usb/usbd_ioreq.cyclo ./Core/Src/usb/usbd_ioreq.d ./Core/Src/usb/usbd_ioreq.o ./Core/Src/usb/usbd_ioreq.su

.PHONY: clean-Core-2f-Src-2f-usb

