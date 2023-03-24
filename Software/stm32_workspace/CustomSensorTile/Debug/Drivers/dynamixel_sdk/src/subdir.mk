################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/dynamixel_sdk/src/group_bulk_read.c \
../Drivers/dynamixel_sdk/src/group_bulk_write.c \
../Drivers/dynamixel_sdk/src/group_sync_read.c \
../Drivers/dynamixel_sdk/src/group_sync_write.c \
../Drivers/dynamixel_sdk/src/packet_handler.c \
../Drivers/dynamixel_sdk/src/port_handler.c \
../Drivers/dynamixel_sdk/src/port_handler_linux.c \
../Drivers/dynamixel_sdk/src/port_handler_mac.c \
../Drivers/dynamixel_sdk/src/port_handler_windows.c \
../Drivers/dynamixel_sdk/src/protocol1_packet_handler.c \
../Drivers/dynamixel_sdk/src/protocol2_packet_handler.c 

OBJS += \
./Drivers/dynamixel_sdk/src/group_bulk_read.o \
./Drivers/dynamixel_sdk/src/group_bulk_write.o \
./Drivers/dynamixel_sdk/src/group_sync_read.o \
./Drivers/dynamixel_sdk/src/group_sync_write.o \
./Drivers/dynamixel_sdk/src/packet_handler.o \
./Drivers/dynamixel_sdk/src/port_handler.o \
./Drivers/dynamixel_sdk/src/port_handler_linux.o \
./Drivers/dynamixel_sdk/src/port_handler_mac.o \
./Drivers/dynamixel_sdk/src/port_handler_windows.o \
./Drivers/dynamixel_sdk/src/protocol1_packet_handler.o \
./Drivers/dynamixel_sdk/src/protocol2_packet_handler.o 

C_DEPS += \
./Drivers/dynamixel_sdk/src/group_bulk_read.d \
./Drivers/dynamixel_sdk/src/group_bulk_write.d \
./Drivers/dynamixel_sdk/src/group_sync_read.d \
./Drivers/dynamixel_sdk/src/group_sync_write.d \
./Drivers/dynamixel_sdk/src/packet_handler.d \
./Drivers/dynamixel_sdk/src/port_handler.d \
./Drivers/dynamixel_sdk/src/port_handler_linux.d \
./Drivers/dynamixel_sdk/src/port_handler_mac.d \
./Drivers/dynamixel_sdk/src/port_handler_windows.d \
./Drivers/dynamixel_sdk/src/protocol1_packet_handler.d \
./Drivers/dynamixel_sdk/src/protocol2_packet_handler.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/dynamixel_sdk/src/%.o Drivers/dynamixel_sdk/src/%.su: ../Drivers/dynamixel_sdk/src/%.c Drivers/dynamixel_sdk/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/dynamixel_sdk/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-dynamixel_sdk-2f-src

clean-Drivers-2f-dynamixel_sdk-2f-src:
	-$(RM) ./Drivers/dynamixel_sdk/src/group_bulk_read.d ./Drivers/dynamixel_sdk/src/group_bulk_read.o ./Drivers/dynamixel_sdk/src/group_bulk_read.su ./Drivers/dynamixel_sdk/src/group_bulk_write.d ./Drivers/dynamixel_sdk/src/group_bulk_write.o ./Drivers/dynamixel_sdk/src/group_bulk_write.su ./Drivers/dynamixel_sdk/src/group_sync_read.d ./Drivers/dynamixel_sdk/src/group_sync_read.o ./Drivers/dynamixel_sdk/src/group_sync_read.su ./Drivers/dynamixel_sdk/src/group_sync_write.d ./Drivers/dynamixel_sdk/src/group_sync_write.o ./Drivers/dynamixel_sdk/src/group_sync_write.su ./Drivers/dynamixel_sdk/src/packet_handler.d ./Drivers/dynamixel_sdk/src/packet_handler.o ./Drivers/dynamixel_sdk/src/packet_handler.su ./Drivers/dynamixel_sdk/src/port_handler.d ./Drivers/dynamixel_sdk/src/port_handler.o ./Drivers/dynamixel_sdk/src/port_handler.su ./Drivers/dynamixel_sdk/src/port_handler_linux.d ./Drivers/dynamixel_sdk/src/port_handler_linux.o ./Drivers/dynamixel_sdk/src/port_handler_linux.su ./Drivers/dynamixel_sdk/src/port_handler_mac.d ./Drivers/dynamixel_sdk/src/port_handler_mac.o ./Drivers/dynamixel_sdk/src/port_handler_mac.su ./Drivers/dynamixel_sdk/src/port_handler_windows.d ./Drivers/dynamixel_sdk/src/port_handler_windows.o ./Drivers/dynamixel_sdk/src/port_handler_windows.su ./Drivers/dynamixel_sdk/src/protocol1_packet_handler.d ./Drivers/dynamixel_sdk/src/protocol1_packet_handler.o ./Drivers/dynamixel_sdk/src/protocol1_packet_handler.su ./Drivers/dynamixel_sdk/src/protocol2_packet_handler.d ./Drivers/dynamixel_sdk/src/protocol2_packet_handler.o ./Drivers/dynamixel_sdk/src/protocol2_packet_handler.su

.PHONY: clean-Drivers-2f-dynamixel_sdk-2f-src

