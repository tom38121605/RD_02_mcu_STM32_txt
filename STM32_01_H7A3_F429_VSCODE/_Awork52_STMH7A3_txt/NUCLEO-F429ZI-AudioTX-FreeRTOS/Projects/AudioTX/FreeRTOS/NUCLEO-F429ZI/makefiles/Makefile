#################################
# Project information
#################################
PROJECT_NAME=NUCLEO-F429ZI-AudioTX-FreeRTOS

#################################
# GNU ARM Embedded Toolchain
#################################
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
NM=arm-none-eabi-nm
SIZE=arm-none-eabi-size
A2L=arm-none-eabi-addr2line


#################################
# Working directories
#################################
BIN_DIR = bin
OBJECT_DIR = obj

# Begin Include list
INCLUDE_DIRS=\
     -I../ProjectDefinition\
     -I../../../../../SDK_BSP/ST/BSP/STM32F4xx_Nucleo_144\
     -I../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Inc\
     -I../../../../../SDK_BSP/ST/CMSIS/Device/ST/STM32F4xx/Include\
     -I../../../../../SDK_BSP/ST/Middlewares/ST/STM32_USB_Device_Library/Core/Inc\
     -I../../../../../SDK_BSP/ST/Middlewares/ST/STM32_Cryptographic/Inc\
     -I../../../../../SDK_BSP/ST/CMSIS/Include\
     -I../../../../../SDK_BSP/ST/Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO_10/Inc\
     -I../../../../../SDK_BSP/ST\
     -I../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS\
     -I../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/Streaming/Inc\
     -I../../../../../Src/Apps/Src/audio\
     -I../../../../../Src/Boards/Inc\
     -I../../../../../SDK_BSP/ST/Middlewares/Third_Party/FreeRTOS/Source/include\
     -I../../../../../SDK_BSP/ST/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F\
     -I../../../../../Src/EventManager\
     -I../../../../../Src/Apps/Src/Inc\
     -I../../../../../Src/Apps/Src/common/app/common\
     -I../../../../../Src/Apps/Src/common/cmd\
     -I../../../../../Src/Apps/Src/common/config\
     -I../../../../../Src/Apps/Src/common/controlTask\
     -I../../../../../Src/Apps/Src/common/defaultTask\
     -I../../../../../Src/Apps/Src/common/flushTask\
     -I../../../../../Src/Apps/Src/common/thread\
     -I../../../../../Src/Apps/Src/common/usb_uart\
     -I../../../../../Src/Apps/Inc\
     -I../../../../../Src/AppConfig/Inc\
     -I../../../../../Src/AppConfig/DefaultConfigs/AudioTX\
     -I../../../../../Src/AppConfig/DefaultConfigs/AudioTX/FreeRTOS\
     -I../../../../../Src/AppConfig/DefaultConfigs/AudioTX/common\
     -I../../../../../Src/Comm/Inc\
     -I../../../../../Src/HAL/Inc\
     -I../../../../../Src/UWB\
     -I../../../../../Src/UWB/pol_dw3000_llhw/include\
     -I../../../../../Libs/uwbstack/llhw_src/include\
     -I../../../../../Libs/uwbstack/llhw_src/llhw_drv/dw3000/include\
     -I../../../../../Src/UWB/FreeRTOS\
     -I../../../../../Src/UWBUtils\
     -I../../../../../Libs/dwt_uwb_driver/Inc\
     -I../../../../../Src/OSAL/Src/FreeRTOS\
     -I../../../../../Src/OSAL/Src/FreeRTOS/CMSIS_FreeRTOS-Legacy\
     -I../../../../../Src/OSAL/Inc\
     -I../../../../../Src/Helpers/Inc\
     -I../../../../../Src/Apps/Src/reporter\
     -I../../../../../Src/Apps/Src/reporter/usb_reporter\
     -I../../../../../Src/Apps/Src/common\
     -I../../../../../Src/Apps/Src/audio/FreeRTOS# End Include list

# Begin Source list
CSOURCES=\
     ../ProjectDefinition/ProjectName.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c\
     ../../../../../SDK_BSP/ST/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c\
     ../../../../../SDK_BSP/ST/BSP/STM32F4xx_Nucleo_144/stm32f4xx_nucleo_144.c\
     ../../../../../SDK_BSP/ST/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c\
     ../../../../../SDK_BSP/ST/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c\
     ../../../../../SDK_BSP/ST/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c\
     ../../../../../SDK_BSP/ST/Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO_10/Src/usbd_audio.c\
     ../../../../../SDK_BSP/ST/Middlewares/Third_Party/FreeRTOS/Source/croutine.c\
     ../../../../../SDK_BSP/ST/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c\
     ../../../../../SDK_BSP/ST/Middlewares/Third_Party/FreeRTOS/Source/list.c\
     ../../../../../SDK_BSP/ST/Middlewares/Third_Party/FreeRTOS/Source/queue.c\
     ../../../../../SDK_BSP/ST/Middlewares/Third_Party/FreeRTOS/Source/tasks.c\
     ../../../../../SDK_BSP/ST/Middlewares/Third_Party/FreeRTOS/Source/timers.c\
     ../../../../../SDK_BSP/ST/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c\
     ../../../../../SDK_BSP/ST/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c\
     ../../../../../Src/EventManager/FreeRTOS/EventManager.c\
     ../../../../../Src/AppConfig/NUCLEO-F429ZI/FreeRTOS/config.c\
     ../../../../../Src/UWBUtils/util.c\
     ../../../../../Src/UWBUtils/translate.c\
     ../../../../../Src/OSAL/Src/FreeRTOS/alloc.c\
     ../../../../../Src/OSAL/Src/FreeRTOS/CMSIS_FreeRTOS-Legacy/cmsis_os.c\
     ../../../../../Src/OSAL/Src/task_signal.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/NUCLEO-F429ZI.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/system_stm32f4xx.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/stm32f4xx_hal_msp.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/stm32f4xx_hal_timebase_tim.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/stm32f4xx_it.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/usbd_conf.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/usbd_desc.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/flash_erase.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/rf_tuning_config.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/Streaming/Src/audio_usb_nodes.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/Streaming/Src/usbd_audio_10_config_descriptors.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/Streaming/Src/usbd_audio_if.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/Streaming/Src/audio_speaker_dw_node.c\
     ../../../../../Src/Boards/Src/NUCLEO-F429ZI/FreeRTOS/Streaming/Src/audio_usb_playback_session.c\
     ../../../../../Src/HAL/Src/STM32/HAL_error.c\
     ../../../../../Src/HAL/Src/STM32/HAL_sleep.c\
     ../../../../../Src/HAL/Src/STM32/HAL_button.c\
     ../../../../../Src/HAL/Src/STM32/HAL_DW3000.c\
     ../../../../../Src/HAL/Src/STM32/HAL_gpio.c\
     ../../../../../Src/HAL/Src/STM32/HAL_RTC.c\
     ../../../../../Src/HAL/Src/STM32/HAL_SPI.c\
     ../../../../../Src/HAL/Src/STM32/HAL_uart.c\
     ../../../../../Src/HAL/Src/STM32F429ZI/HAL_timer.c\
     ../../../../../Src/HAL/Src/STM32F429ZI/HAL_usb.c\
     ../../../../../Src/HAL/Src/STM32F429ZI/HAL_watchdog.c\
     ../../../../../Src/Apps/Src/common/app/common/app.c\
     ../../../../../Src/Apps/Src/common/config/driver_app_config.c\
     ../../../../../Src/Apps/Src/common/config/debug_config.c\
     ../../../../../Src/Apps/Src/common/controlTask/controlTask.c\
     ../../../../../Src/Apps/Src/common/controlTask/FreeRTOS/create_control_task.c\
     ../../../../../Src/Apps/Src/common/flushTask/flushTask.c\
     ../../../../../Src/Apps/Src/common/flushTask/FreeRTOS/create_flush_task.c\
     ../../../../../Src/Apps/Src/common/usb_uart/usb_uart_tx.c\
     ../../../../../Src/Apps/Src/common/usb_uart/usb_uart_rx.c\
     ../../../../../Src/Apps/Src/common/thread/FreeRTOS/thread_fn.c\
     ../../../../../Src/Apps/Src/common/cmd/cmd.c\
     ../../../../../Src/Apps/Src/common/cmd/cmd_fn.c\
     ../../../../../Src/Apps/Src/common/defaultTask/FreeRTOS/create_default_task.c\
     ../../../../../Src/Apps/Src/common/defaultTask/defaultTask.c\
     ../../../../../Src/Apps/Src/common/cmd/cmd_rf_tuning.c\
     ../../../../../Src/Apps/Src/common/cmd/cmd_dw3000.c\
     ../../../../../Src/UWB/stubs.c\
     ../../../../../Src/Apps/Src/reporter/usb_reporter/usb_reporter.c\
     ../../../../../Src/Comm/Src/InterfUsb.c\
     ../../../../../Src/Comm/Src/comm_config.c\
     ../../../../../Src/Apps/Src/audio/audio_i2s_speaker.c\
     ../../../../../Src/Apps/Src/audio/audio_tx.c\
     ../../../../../Src/Apps/Src/audio/task_audio_tx.c\
     ../../../../../Src/Apps/Src/audio/audio_tx_fn.c\
     ../../../../../Src/Apps/Src/audio/common_audio.c\
     ../../../../../Src/Apps/Src/audio/FreeRTOS/create_audio_tx_task.c\
     ../../../../../Src/Apps/Src/audio/FreeRTOS/create_audio_i2s_task.c\
     ../../../../../Src/Apps/Src/audio/audio_dw3000.c\
     ../../../../../Src/Helpers/crc16.c\
     ../../../../../Src/Helpers/cJSON.c\
     ../../../../../Src/Helpers/deca_dbg.c\
     ../../AudioTX-FreeRTOS-Common/main.c\
     ../../AudioTX-FreeRTOS-Common/hooks.c# End Source list

# Begin Assembly list
ASOURCES=\
     ../../../../../SDK_BSP/ST/startup/startup_stm32f429xx.s# End Assembly list

# Begin STATICLIB list
STATICLIB=\
     ../../../../../Libs/dwt_uwb_driver/lib/libdwt_uwb_driver-m4-hfp-7.10.1.a# End STATICLIB list

#################################
# Object List
#################################
OBJECTS = $(addprefix $(OBJECT_DIR)/,$(notdir $(CSOURCES:.c=.o)))
vpath %.c $(sort $(dir $(CSOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(OBJECT_DIR)/,$(notdir $(ASOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASOURCES)))
#################################
# Target Output Files
#################################
TARGET_ELF=$(BIN_DIR)/$(PROJECT_NAME).elf
TARGET_HEX=$(BIN_DIR)/$(PROJECT_NAME).hex
TARGET_BIN=$(BIN_DIR)/$(PROJECT_NAME).bin

#######################################
# CFLAGS
#######################################

# Begin CFLAGS list
CFLAGS=\
     -DCLI_BUILD\
     -DUSE_DRV_DW3000\
     -DUSE_DRV_DW3720\
     -DNB_DW_DRIVERS=2\
     -DPROJECT_NAME=\"AudioTX\"\
     -DDMA_SPI\
     -DUSB_ENABLE\
     -DUSE_USB_FS\
     -DUWB_DATA_TRANS_DEBUG_TX=0\
     -DUSE_I2S_AUDIO_PLAYBACK=0\
     -DUSE_USB_AUDIO_PLAYBACK=1\
     -DUSE_AUDIO_TX=1\
     -DUSE_AUDIO=1\
     -O0\
     -Werror\
     -DSTM32F429ZI\
     -g3\
     -Wall\
     -std=c11\
     -ffunction-sections\
     -fdata-sections\
     -specs=nosys.specs\
     -specs=nano.specs\
     -mthumb\
     -mfloat-abi=hard\
     -mfpu=fpv4-sp-d16\
     -mcpu=cortex-m4# End CFLAGS list

# Begin ASMFLAGS list
ASMFLAGS=\
     -DDMA_SPI\
     -DUSB_ENABLE\
     -DUSE_USB_FS\
     -DUWB_DATA_TRANS_DEBUG_TX=0\
     -DUSE_I2S_AUDIO_PLAYBACK=0\
     -DUSE_USB_AUDIO_PLAYBACK=1\
     -DUSE_AUDIO_TX=1\
     -DUSE_AUDIO=1\
     -O0\
     -Werror\
     -D__HEAP_SIZE=0\
     -D__STACK_SIZE=2048\
     -DSTM32F429ZI\
     -g3\
     -Wall\
     -ffunction-sections\
     -fdata-sections\
     -mthumb\
     -mfloat-abi=hard\
     -mfpu=fpv4-sp-d16\
     -mcpu=cortex-m4# End ASMFLAGS list

# Begin LDFLAGS list
LDFLAGS=\
	-lc \
	-lm \
	-lnosys \
	-z muldefs\
     -Wl,--gc-sections,-Map=$(PROJECT_NAME).map\
     -u _printf_float\
# End LDFLAGS list

# Begin LINKERFILE list
LINKERFILE=\
     ../fw3.ld# End LINKERFILE list

#################################
# Recipes
#################################
.PHONY: all clean

all: prepare target

target: $(TARGET_ELF) $(TARGET_HEX) $(TARGET_BIN)

clean:
	-rm -Rf $(OBJECTS) $(OBJECT_DIR) $(TARGET_ELF) $(BIN_DIR)

prepare:
	@mkdir -p $(OBJECT_DIR)
	@mkdir -p $(BIN_DIR)


$(OBJECT_DIR)/%.o: %.c
	@$(CC) -c -o $@ $(CFLAGS) $(INCLUDE_DIRS) $<

$(OBJECT_DIR)/%.o: %.s
	@$(CC) -c -o $@ $(ASMFLAGS) $<

$(TARGET_ELF): $(OBJECTS)
	@$(CC) $(OBJECTS) $(CFLAGS) $(LDFLAGS) -Wl,-T${LINKERFILE} -Wl,--whole-archive ${STATICLIB} -Wl,--no-whole-archive -o $@
	$(SIZE) $@

$(TARGET_HEX): $(TARGET_ELF)
	$(CP) -O ihex $< $@

$(TARGET_BIN): $(TARGET_ELF)
	$(CP) -O binary -S $< $@

