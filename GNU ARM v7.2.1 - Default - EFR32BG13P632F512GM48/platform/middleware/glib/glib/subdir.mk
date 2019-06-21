################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../platform/middleware/glib/glib/bmp.c \
../platform/middleware/glib/glib/glib.c \
../platform/middleware/glib/glib/glib_bitmap.c \
../platform/middleware/glib/glib/glib_circle.c \
../platform/middleware/glib/glib/glib_font_narrow_6x8.c \
../platform/middleware/glib/glib/glib_font_normal_8x8.c \
../platform/middleware/glib/glib/glib_font_number_16x20.c \
../platform/middleware/glib/glib/glib_line.c \
../platform/middleware/glib/glib/glib_polygon.c \
../platform/middleware/glib/glib/glib_rectangle.c \
../platform/middleware/glib/glib/glib_string.c 

OBJS += \
./platform/middleware/glib/glib/bmp.o \
./platform/middleware/glib/glib/glib.o \
./platform/middleware/glib/glib/glib_bitmap.o \
./platform/middleware/glib/glib/glib_circle.o \
./platform/middleware/glib/glib/glib_font_narrow_6x8.o \
./platform/middleware/glib/glib/glib_font_normal_8x8.o \
./platform/middleware/glib/glib/glib_font_number_16x20.o \
./platform/middleware/glib/glib/glib_line.o \
./platform/middleware/glib/glib/glib_polygon.o \
./platform/middleware/glib/glib/glib_rectangle.o \
./platform/middleware/glib/glib/glib_string.o 

C_DEPS += \
./platform/middleware/glib/glib/bmp.d \
./platform/middleware/glib/glib/glib.d \
./platform/middleware/glib/glib/glib_bitmap.d \
./platform/middleware/glib/glib/glib_circle.d \
./platform/middleware/glib/glib/glib_font_narrow_6x8.d \
./platform/middleware/glib/glib/glib_font_normal_8x8.d \
./platform/middleware/glib/glib/glib_font_number_16x20.d \
./platform/middleware/glib/glib/glib_line.d \
./platform/middleware/glib/glib/glib_polygon.d \
./platform/middleware/glib/glib/glib_rectangle.d \
./platform/middleware/glib/glib/glib_string.d 


# Each subdirectory must supply rules for building sources it contributes
platform/middleware/glib/glib/bmp.o: ../platform/middleware/glib/glib/bmp.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\halconfig" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\bsp" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\CMSIS\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\drivers" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\bootloader\api" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\common\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/middleware/glib/glib/bmp.d" -MT"platform/middleware/glib/glib/bmp.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/middleware/glib/glib/glib.o: ../platform/middleware/glib/glib/glib.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\halconfig" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\bsp" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\CMSIS\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\drivers" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\bootloader\api" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\common\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/middleware/glib/glib/glib.d" -MT"platform/middleware/glib/glib/glib.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/middleware/glib/glib/glib_bitmap.o: ../platform/middleware/glib/glib/glib_bitmap.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\halconfig" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\bsp" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\CMSIS\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\drivers" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\bootloader\api" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\common\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/middleware/glib/glib/glib_bitmap.d" -MT"platform/middleware/glib/glib/glib_bitmap.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/middleware/glib/glib/glib_circle.o: ../platform/middleware/glib/glib/glib_circle.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\halconfig" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\bsp" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\CMSIS\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\drivers" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\bootloader\api" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\common\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/middleware/glib/glib/glib_circle.d" -MT"platform/middleware/glib/glib/glib_circle.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/middleware/glib/glib/glib_font_narrow_6x8.o: ../platform/middleware/glib/glib/glib_font_narrow_6x8.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\halconfig" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\bsp" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\CMSIS\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\drivers" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\bootloader\api" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\common\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/middleware/glib/glib/glib_font_narrow_6x8.d" -MT"platform/middleware/glib/glib/glib_font_narrow_6x8.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/middleware/glib/glib/glib_font_normal_8x8.o: ../platform/middleware/glib/glib/glib_font_normal_8x8.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\halconfig" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\bsp" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\CMSIS\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\drivers" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\bootloader\api" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\common\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/middleware/glib/glib/glib_font_normal_8x8.d" -MT"platform/middleware/glib/glib/glib_font_normal_8x8.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/middleware/glib/glib/glib_font_number_16x20.o: ../platform/middleware/glib/glib/glib_font_number_16x20.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\halconfig" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\bsp" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\CMSIS\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\drivers" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\bootloader\api" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\common\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/middleware/glib/glib/glib_font_number_16x20.d" -MT"platform/middleware/glib/glib/glib_font_number_16x20.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/middleware/glib/glib/glib_line.o: ../platform/middleware/glib/glib/glib_line.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\halconfig" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\bsp" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\CMSIS\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\drivers" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\bootloader\api" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\common\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/middleware/glib/glib/glib_line.d" -MT"platform/middleware/glib/glib/glib_line.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/middleware/glib/glib/glib_polygon.o: ../platform/middleware/glib/glib/glib_polygon.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\halconfig" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\bsp" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\CMSIS\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\drivers" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\bootloader\api" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\common\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/middleware/glib/glib/glib_polygon.d" -MT"platform/middleware/glib/glib/glib_polygon.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/middleware/glib/glib/glib_rectangle.o: ../platform/middleware/glib/glib/glib_rectangle.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\halconfig" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\bsp" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\CMSIS\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\drivers" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\bootloader\api" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\common\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/middleware/glib/glib/glib_rectangle.d" -MT"platform/middleware/glib/glib/glib_rectangle.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/middleware/glib/glib/glib_string.o: ../platform/middleware/glib/glib/glib_string.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emlib\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\halconfig" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\bsp" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\CMSIS\Include" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\common\drivers" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\sleep\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\bootloader\api" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\emdrv\common\inc" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\glib" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\Desktop\Mesh_Receive_data-master\platform\middleware\glib\dmd" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/middleware/glib/glib/glib_string.d" -MT"platform/middleware/glib/glib/glib_string.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


