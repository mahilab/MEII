add_executable(mahiexoii_pos_ctrl ex_mahiexoii_pos_ctrl.cpp)
target_link_libraries(mahiexoii_pos_ctrl meii::meii)

add_executable(ex_pos_control_nathan_rob ex_pos_control_nathan_rob.cpp)
target_link_libraries(ex_pos_control_nathan_rob meii::meii)

add_executable(dynamic_motion_primitive ex_dynamic_motion_primitive.cpp)
target_link_libraries(dynamic_motion_primitive meii::meii)

add_executable(virtual_rom_demo ex_virtual_rom_demo.cpp)
target_link_libraries(virtual_rom_demo meii::meii)

add_executable(virtual_rom_filter ex_virtual_rom_filter.cpp)
target_link_libraries(virtual_rom_filter meii::meii)