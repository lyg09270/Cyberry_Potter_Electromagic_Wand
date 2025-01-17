cmake_minimum_required(VERSION 3.27)

# Roots
include("../roots.cmake")

set(CONTEXT CyberryPotter+Target_1)
set(TARGET ${CONTEXT})
set(OUT_DIR "${SOLUTION_ROOT}/out/CyberryPotter/Target_1")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(LD_SCRIPT "${SOLUTION_ROOT}/CyberryPotter_Target_1.sct")
set(LD_SCRIPT_PP ${LD_SCRIPT})

# Processor Options
set(CPU Cortex-M3)
set(FPU NO_FPU)
set(BYTE_ORDER Little-endian)

# Toolchain config map
include("toolchain.cmake")

# Setup project
project(${CONTEXT} LANGUAGES C ASM)

# Compilation database
add_custom_target(database
  COMMAND ${CMAKE_COMMAND} -E make_directory "${OUT_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json" "${OUT_DIR}"
)
set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES})

# Setup context
add_executable(${CONTEXT})
set_target_properties(${CONTEXT} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "CyberryPotter")
set_target_properties(${CONTEXT} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR})
add_library(${CONTEXT}_GLOBAL INTERFACE)

# Includes
target_include_directories(${CONTEXT} PUBLIC
  ${SOLUTION_ROOT}/RTE/_Target_1
  ${SOLUTION_ROOT}/RTE/Device/STM32F103CB
  ${CMSIS_PACK_ROOT}/ARM/CMSIS/6.0.0/CMSIS/Core/Include
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/Include
  ${CMSIS_PACK_ROOT}/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/inc
  ${SOLUTION_ROOT}/CNN
  ${SOLUTION_ROOT}/CNN/nnom_lib/inc
  ${SOLUTION_ROOT}/CNN/nnom_lib/inc/layers
  ${SOLUTION_ROOT}/CNN/nnom_lib/port
  ${SOLUTION_ROOT}/MainBoard
  ${SOLUTION_ROOT}/MainBoard/eMPL
  ${SOLUTION_ROOT}/Module
)

# Defines
target_compile_definitions(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:C,CXX>:
    USE_STDPERIPH_DRIVER
    __MICROLIB
    STM32F10X_MD
    _RTE_
  >
)

# Compile options
target_compile_options(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:ASM>:
    "SHELL:${ASM_CPU}"
    "SHELL:${ASM_FLAGS}"
    "SHELL:${ASM_BYTE_ORDER}"
  >
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_CPU}"
    "SHELL:${CC_FLAGS}"
    "SHELL:${CC_BYTE_ORDER}"
    "SHELL:-std=c99"
    "SHELL:-Wno-nan-infinity-disabled"
    "SHELL:-fno-rtti"
    "SHELL:-funsigned-char"
    "SHELL:-fno-function-sections"
    "SHELL:-Wno-packed"
    "SHELL:-Wno-missing-variable-declarations"
    "SHELL:-Wno-missing-prototypes"
    "SHELL:-Wno-missing-noreturn"
    "SHELL:-Wno-sign-conversion"
    "SHELL:-Wno-nonportable-include-path"
    "SHELL:-Wno-reserved-id-macro"
    "SHELL:-Wno-unused-macros"
    "SHELL:-Wno-documentation-unknown-command"
    "SHELL:-Wno-documentation"
    "SHELL:-Wno-license-management"
    "SHELL:-Wno-parentheses-equality"
  >
)

# Compile Options Abstractions
add_library(${CONTEXT}_ABSTRACTIONS INTERFACE)
cbuild_set_options_flags(CC "none" "on" "" "" CC_OPTIONS_FLAGS_${CONTEXT})
cbuild_set_options_flags(ASM "none" "on" "" "" ASM_OPTIONS_FLAGS_${CONTEXT})
target_compile_options(${CONTEXT}_ABSTRACTIONS INTERFACE
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_OPTIONS_FLAGS_${CONTEXT}}"
  >
  $<$<COMPILE_LANGUAGE:ASM>:
    "SHELL:${ASM_OPTIONS_FLAGS_${CONTEXT}}"
  >
)

# Add groups and components
include("groups.cmake")
include("components.cmake")

target_link_libraries(${CONTEXT} PUBLIC
  Group_nnom
  Group_MainBoard
  Group_Module
  ARM_CMSIS_CORE_6_0_0
  Keil_Device_Startup_1_0_0
  Keil_Device_StdPeriph_Drivers_ADC_3_6_0
  Keil_Device_StdPeriph_Drivers_BKP_3_6_0
  Keil_Device_StdPeriph_Drivers_CAN_3_6_0
  Keil_Device_StdPeriph_Drivers_CEC_3_6_0
  Keil_Device_StdPeriph_Drivers_CRC_3_6_0
  Keil_Device_StdPeriph_Drivers_DAC_3_6_0
  Keil_Device_StdPeriph_Drivers_DBGMCU_3_6_0
  Keil_Device_StdPeriph_Drivers_DMA_3_6_0
  Keil_Device_StdPeriph_Drivers_EXTI_3_6_0
  Keil_Device_StdPeriph_Drivers_FSMC_3_6_0
  Keil_Device_StdPeriph_Drivers_Flash_3_6_0
  Keil_Device_StdPeriph_Drivers_Framework_3_6_0
  Keil_Device_StdPeriph_Drivers_GPIO_3_6_0
  Keil_Device_StdPeriph_Drivers_I2C_3_6_0
  Keil_Device_StdPeriph_Drivers_IWDG_3_6_0
  Keil_Device_StdPeriph_Drivers_PWR_3_6_0
  Keil_Device_StdPeriph_Drivers_RCC_3_6_0
  Keil_Device_StdPeriph_Drivers_RTC_3_6_0
  Keil_Device_StdPeriph_Drivers_SDIO_3_6_0
  Keil_Device_StdPeriph_Drivers_SPI_3_6_0
  Keil_Device_StdPeriph_Drivers_TIM_3_6_0
  Keil_Device_StdPeriph_Drivers_USART_3_6_0
  Keil_Device_StdPeriph_Drivers_WWDG_3_6_0
)

# Linker options
target_link_options(${CONTEXT} PUBLIC
  "SHELL:${LD_CPU}"
  "SHELL:${_LS}\"${LD_SCRIPT_PP}\""
  "SHELL:--library_type=microlib"
  "SHELL:--map"
  "SHELL:--load_addr_map_info"
  "SHELL:--xref"
  "SHELL:--callgraph"
  "SHELL:--symbols"
  "SHELL:--info sizes"
  "SHELL:--info totals"
  "SHELL:--info unused"
  "SHELL:--info veneers"
  "SHELL:--strict"
  "SHELL:--summary_stderr"
  "SHELL:--info summarysizes"
)
set_target_properties(${CONTEXT} PROPERTIES LINK_DEPENDS ${LD_SCRIPT})
