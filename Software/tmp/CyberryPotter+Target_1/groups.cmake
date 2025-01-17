# groups.cmake

# group nnom
add_library(Group_nnom OBJECT
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/backends/nnom_local.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/backends/nnom_local_q15.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/core/nnom.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/core/nnom_layers.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/core/nnom_tensor.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/core/nnom_utils.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_activation.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_avgpool.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_baselayer.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_concat.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_conv2d.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_conv2d_trans.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_cropping.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_dense.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_dw_conv2d.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_flatten.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_global_pool.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_gru_cell.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_input.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_lambda.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_lstm_cell.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_matrix.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_maxpool.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_output.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_reshape.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_rnn.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_simple_cell.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_softmax.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_sumpool.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_upsample.c"
  "${SOLUTION_ROOT}/CNN/nnom_lib/src/layers/nnom_zero_padding.c"
)
target_include_directories(Group_nnom PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${SOLUTION_ROOT}/CNN
  ${SOLUTION_ROOT}/CNN/nnom_lib/inc
  ${SOLUTION_ROOT}/CNN/nnom_lib/inc/layers
  ${SOLUTION_ROOT}/CNN/nnom_lib/port
)
target_compile_definitions(Group_nnom PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Group_nnom PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_nnom PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# group MainBoard
add_library(Group_MainBoard OBJECT
  "${SOLUTION_ROOT}/MainBoard/CyberryPotter.c"
  "${SOLUTION_ROOT}/MainBoard/Delay.c"
  "${SOLUTION_ROOT}/MainBoard/IIC.c"
  "${SOLUTION_ROOT}/MainBoard/IMU.c"
  "${SOLUTION_ROOT}/MainBoard/main.c"
  "${SOLUTION_ROOT}/MainBoard/MPU6050.c"
  "${SOLUTION_ROOT}/MainBoard/W25Q64.c"
  "${SOLUTION_ROOT}/MainBoard/eMPL/inv_mpu.c"
  "${SOLUTION_ROOT}/MainBoard/eMPL/inv_mpu_dmp_motion_driver.c"
  "${SOLUTION_ROOT}/MainBoard/SPI.c"
  "${SOLUTION_ROOT}/MainBoard/USART.c"
  "${SOLUTION_ROOT}/MainBoard/button.c"
  "${SOLUTION_ROOT}/MainBoard/LED.c"
  "${SOLUTION_ROOT}/MainBoard/ADC.c"
)
target_include_directories(Group_MainBoard PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${SOLUTION_ROOT}/MainBoard
  ${SOLUTION_ROOT}/MainBoard/eMPL
)
target_compile_definitions(Group_MainBoard PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Group_MainBoard PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_MainBoard PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)

# group Module
add_library(Group_Module OBJECT
  "${SOLUTION_ROOT}/Module/module0_IR.c"
  "${SOLUTION_ROOT}/Module/module1_RF433.c"
  "${SOLUTION_ROOT}/Module/module2_RF315.c"
  "${SOLUTION_ROOT}/Module/module_IR_RF.c"
  "${SOLUTION_ROOT}/Module/module3.c"
  "${SOLUTION_ROOT}/Module/module4.c"
  "${SOLUTION_ROOT}/Module/module5.c"
  "${SOLUTION_ROOT}/Module/module6.c"
  "${SOLUTION_ROOT}/Module/module7.c"
  "${SOLUTION_ROOT}/Module/module8.c"
  "${SOLUTION_ROOT}/Module/module9.c"
  "${SOLUTION_ROOT}/Module/module10.c"
)
target_include_directories(Group_Module PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  ${SOLUTION_ROOT}/Module
)
target_compile_definitions(Group_Module PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Group_Module PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Module PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)
