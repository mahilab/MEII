add_executable(ex_trajectory ex_trajectory.cpp)
target_link_libraries(ex_trajectory meii)

add_executable(ex_mahiexoii_pos_ctrl ex_mahiexoii_pos_ctrl.cpp)
target_link_libraries(ex_mahiexoii_pos_ctrl meii)

add_executable(ex_pos_control_nathan ex_pos_control_nathan.cpp)
target_link_libraries(ex_pos_control_nathan meii)

# add_executable(ex_pos_control_nathan_rob ex_pos_control_nathan_rob.cpp)
# target_link_libraries(ex_pos_control_nathan_rob meii)

# add_executable(ex_dist_observer ex_dist_observer.cpp)
# target_link_libraries(ex_dist_observer meii)

add_executable(ex_rom_demo ex_rom_demo.cpp)
target_link_libraries(ex_rom_demo meii)