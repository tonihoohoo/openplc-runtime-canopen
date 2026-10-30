# SPDX-License-Identifier: Apache-2.0

add_library(canopen_sim STATIC ${CMAKE_CURRENT_SOURCE_DIR}/../tools/sim_slave.cpp)
target_include_directories(canopen_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../tools)
target_link_libraries(canopen_sim PUBLIC canopen_core)

foreach(name test_bus_off_recovery test_can_link test_config test_link_stall test_master_dcf test_network test_status)
    add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${name}.cpp)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wno-unused-parameter)
    target_link_libraries(${name} PRIVATE canopen_core canopen_sim)
    add_test(NAME ${name} COMMAND ${name})
endforeach()
set_tests_properties(test_network PROPERTIES TIMEOUT 120)

add_executable(test_entry_points ${CMAKE_CURRENT_SOURCE_DIR}/test_entry_points.cpp
                                 ${CMAKE_CURRENT_SOURCE_DIR}/../canopen_plugin.cpp)
target_compile_options(test_entry_points PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(test_entry_points PRIVATE canopen_core)
add_test(NAME test_entry_points COMMAND test_entry_points)
set_tests_properties(test_entry_points PROPERTIES TIMEOUT 300)

add_executable(canopen_sim_slave ${CMAKE_CURRENT_SOURCE_DIR}/../tools/sim_slave_main.cpp)
target_link_libraries(canopen_sim_slave PRIVATE canopen_sim)
