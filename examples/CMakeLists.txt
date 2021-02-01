add_executable(mahiexoii_pos_ctrl ex_mahiexoii_pos_ctrl.cpp)
target_link_libraries(mahiexoii_pos_ctrl meii::meii)

add_executable(pos_control_nathan ex_pos_control_nathan.cpp)
target_link_libraries(pos_control_nathan meii::meii)

add_executable(ex_pos_control_nathan_rob ex_pos_control_nathan_rob.cpp)
target_link_libraries(ex_pos_control_nathan_rob meii::meii)

add_executable(dynamic_motion_primitive ex_dynamic_motion_primitive.cpp)
target_link_libraries(dynamic_motion_primitive meii::meii)

# add_executable(rom_demo ex_rom_demo.cpp)
# target_link_libraries(rom_demo meii::meii)

add_executable(virtual_rom_demo ex_virtual_rom_demo.cpp)
target_link_libraries(virtual_rom_demo meii::meii)

add_executable(virtual_rom_filter ex_virtual_rom_filter.cpp)
target_link_libraries(virtual_rom_filter meii::meii)

add_executable(qpid_test ex_qpid_test.cpp)
target_link_libraries(qpid_test meii::meii)