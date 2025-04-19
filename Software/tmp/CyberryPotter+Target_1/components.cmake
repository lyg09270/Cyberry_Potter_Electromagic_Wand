# components.cmake

# component ARM::CMSIS:CORE@6.0.0
add_library(ARM_CMSIS_CORE_6_0_0 INTERFACE)
target_include_directories(ARM_CMSIS_CORE_6_0_0 INTERFACE
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/ARM/CMSIS/6.0.0/CMSIS/Core/Include
)
target_compile_definitions(ARM_CMSIS_CORE_6_0_0 INTERFACE
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_link_libraries(ARM_CMSIS_CORE_6_0_0 INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:Startup@1.0.0
add_library(Keil_Device_Startup_1_0_0 OBJECT
  "${SOLUTION_ROOT}/RTE/Device/STM32F103CB/startup_stm32f10x_md.s"
  "${SOLUTION_ROOT}/RTE/Device/STM32F103CB/system_stm32f10x.c"
)
target_include_directories(Keil_Device_Startup_1_0_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${SOLUTION_ROOT}/RTE/Device/STM32F103CB
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/Include
)
target_compile_definitions(Keil_Device_Startup_1_0_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_Startup_1_0_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
  $<$<COMPILE_LANGUAGE:ASM>:
    "SHELL:-masm=auto"
  >
)
target_link_libraries(Keil_Device_Startup_1_0_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)
set(COMPILE_DEFINITIONS
  __MICROLIB
  STM32F10X_MD
  _RTE_
)
cbuild_set_defines(AS_ARM COMPILE_DEFINITIONS)
set_source_files_properties("${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/Source/ARM/STM32F1xx_OPT.s" PROPERTIES
  COMPILE_FLAGS "${COMPILE_DEFINITIONS}"
)
set(COMPILE_DEFINITIONS
  __MICROLIB
  STM32F10X_MD
  _RTE_
)
cbuild_set_defines(AS_ARM COMPILE_DEFINITIONS)
set_source_files_properties("${SOLUTION_ROOT}/RTE/Device/STM32F103CB/startup_stm32f10x_md.s" PROPERTIES
  COMPILE_FLAGS "${COMPILE_DEFINITIONS}"
)

# component Keil::Device:StdPeriph Drivers:ADC@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_ADC_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_adc.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_ADC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_ADC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_ADC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_ADC_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:BKP@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_BKP_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_bkp.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_BKP_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_BKP_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_BKP_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_BKP_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:CAN@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_CAN_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_can.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_CAN_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_CAN_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_CAN_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_CAN_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:CEC@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_CEC_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_cec.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_CEC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_CEC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_CEC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_CEC_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:CRC@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_CRC_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_crc.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_CRC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_CRC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_CRC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_CRC_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:DAC@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_DAC_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_dac.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_DAC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_DAC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_DAC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_DAC_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:DBGMCU@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_DBGMCU_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_dbgmcu.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_DBGMCU_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_DBGMCU_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_DBGMCU_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_DBGMCU_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:DMA@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_DMA_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_dma.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_DMA_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_DMA_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_DMA_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_DMA_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:EXTI@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_EXTI_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_exti.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_EXTI_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_EXTI_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_EXTI_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_EXTI_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:FSMC@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_FSMC_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_fsmc.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_FSMC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_FSMC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_FSMC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_FSMC_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:Flash@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_Flash_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_flash.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_Flash_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_Flash_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_Flash_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_Flash_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:Framework@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_Framework_3_6_0 OBJECT
  "${SOLUTION_ROOT}/RTE/Device/STM32F103CB/stm32f10x_conf.h"
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/misc.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_Framework_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_Framework_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_Framework_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_Framework_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:GPIO@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_GPIO_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_gpio.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_GPIO_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_GPIO_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_GPIO_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_GPIO_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:I2C@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_I2C_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_i2c.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_I2C_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_I2C_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_I2C_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_I2C_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:IWDG@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_IWDG_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_iwdg.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_IWDG_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_IWDG_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_IWDG_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_IWDG_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:PWR@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_PWR_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_pwr.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_PWR_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_PWR_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_PWR_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_PWR_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:RCC@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_RCC_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_rcc.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_RCC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_RCC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_RCC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_RCC_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:RTC@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_RTC_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_rtc.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_RTC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_RTC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_RTC_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_RTC_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:SDIO@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_SDIO_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_sdio.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_SDIO_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_SDIO_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_SDIO_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_SDIO_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:SPI@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_SPI_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_spi.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_SPI_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_SPI_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_SPI_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_SPI_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:TIM@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_TIM_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_tim.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_TIM_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_TIM_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_TIM_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_TIM_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:USART@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_USART_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_usart.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_USART_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_USART_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_USART_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_USART_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:StdPeriph Drivers:WWDG@3.6.0
add_library(Keil_Device_StdPeriph_Drivers_WWDG_3_6_0 OBJECT
  "${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/stm32f10x_wwdg.c"
)
target_include_directories(Keil_Device_StdPeriph_Drivers_WWDG_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
)
target_compile_definitions(Keil_Device_StdPeriph_Drivers_WWDG_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_StdPeriph_Drivers_WWDG_3_6_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_StdPeriph_Drivers_WWDG_3_6_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)
