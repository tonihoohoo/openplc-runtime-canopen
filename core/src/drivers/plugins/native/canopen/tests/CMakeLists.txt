# SPDX-License-Identifier: Apache-2.0

add_library(canopen_sim STATIC ${CMAKE_CURRENT_SOURCE_DIR}/../tools/sim_slave.cpp)
target_include_directories(canopen_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../tools)
target_link_libraries(canopen_sim PUBLIC canopen_core)

foreach(name test_config test_master_dcf test_network)
    add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${name}.cpp)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wno-unused-parameter)
    target_link_libraries(${name} PRIVATE canopen_core canopen_sim)
    add_test(NAME ${name} COMMAND ${name})
endforeach()
set_tests_properties(test_network PROPERTIES TIMEOUT 120)

add_executable(canopen_sim_slave ${CMAKE_CURRENT_SOURCE_DIR}/../tools/sim_slave_main.cpp)
target_link_libraries(canopen_sim_slave PRIVATE canopen_sim)
