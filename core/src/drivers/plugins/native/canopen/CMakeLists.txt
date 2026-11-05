# SPDX-License-Identifier: Apache-2.0
#
# CANopen plugin: a CANopen master on SocketCAN, built on Lely CANopen (Apache-2.0), which is
# fetched at a pinned commit and linked statically into the plugin, and a CANopen slave built on
# CANopenNode (Apache-2.0), fetched at its newest release and compiled in.
#
# -DCANOPENNODE_SOURCE_DIR=<dir> use that CANopenNode checkout instead of fetching it
# -DLELY_PREFIX=<dir>      use a Lely already installed there instead of building it; a prefix in
#                          lely-prefix/ next to this file is used by default (the release workflow
#                          puts a cached build there)
# -DCANOPEN_BUILD_TESTS=ON also build the unit and host tests (ctest)

cmake_minimum_required(VERSION 3.10)
project(canopen_plugin C CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "CANopen plugin: SocketCAN is Linux only, skipped on ${CMAKE_SYSTEM_NAME}")
    return()
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT DEFINED OPENPLC_ROOT)
    get_filename_component(OPENPLC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../" ABSOLUTE)
endif()

option(CANOPEN_BUILD_TESTS "Build the CANopen plugin tests" OFF)
set(LELY_PREFIX "" CACHE PATH "Installed Lely CANopen to use instead of building it")
if(NOT LELY_PREFIX AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lely-prefix/lib/liblely-co.a)
    set(LELY_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/lely-prefix)
    message(STATUS "CANopen plugin: using the Lely build in ${LELY_PREFIX}")
endif()

set(LELY_LIBS coapp co io2 ev can util libc)

if(LELY_PREFIX)
    set(LELY_INCLUDE_DIR ${LELY_PREFIX}/include)
    set(LELY_LIB_DIR ${LELY_PREFIX}/lib)
else()
    include(ExternalProject)
    set(LELY_GIT_URL "https://gitlab.com/lely_industries/lely-core.git")
    set(LELY_GIT_TAG "88848aa28599ea5d6d7e766a9ffa2054446821b5")
    set(LELY_INSTALL ${CMAKE_CURRENT_BINARY_DIR}/lely)
    set(LELY_INCLUDE_DIR ${LELY_INSTALL}/include)
    set(LELY_LIB_DIR ${LELY_INSTALL}/lib)
    set(LELY_BYPRODUCTS "")
    foreach(lib ${LELY_LIBS})
        list(APPEND LELY_BYPRODUCTS ${LELY_LIB_DIR}/liblely-${lib}.a)
    endforeach()
    ExternalProject_Add(lely
        GIT_REPOSITORY ${LELY_GIT_URL}
        GIT_TAG ${LELY_GIT_TAG}
        GIT_SHALLOW OFF
        UPDATE_COMMAND ""
        BUILD_IN_SOURCE ON
        CONFIGURE_COMMAND autoreconf -i
            COMMAND ./configure --prefix=${LELY_INSTALL} --disable-shared --enable-static --with-pic
                --disable-python --disable-tests --disable-unit-tests --disable-tools
                --disable-daemon --disable-gw --disable-gw-txt CPPFLAGS=-DNDEBUG
        BUILD_COMMAND $(MAKE)
        INSTALL_COMMAND $(MAKE) install
        BUILD_BYPRODUCTS ${LELY_BYPRODUCTS}
    )
    file(MAKE_DIRECTORY ${LELY_INCLUDE_DIR})
endif()

# CANopenNode (Apache-2.0) for the slave, at its newest release, compiled into the plugin with
# the slave's own target header (slave/CO_driver_target.h). Without it the plugin builds without
# the slave and refuses a configuration with devices.
set(CANOPENNODE_SOURCE_DIR "" CACHE PATH "CANopenNode checkout to use instead of fetching it")
set(CANOPENNODE_GIT_URL "https://github.com/CANopenNode/CANopenNode.git")
set(CANOPENNODE_GIT_TAG "v4.1")
if(NOT CANOPENNODE_SOURCE_DIR)
    set(CANOPENNODE_SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/CANopenNode)
    if(NOT EXISTS ${CANOPENNODE_SOURCE_DIR}/CANopen.c)
        execute_process(
            COMMAND git clone --quiet --depth 1 --branch ${CANOPENNODE_GIT_TAG}
                    ${CANOPENNODE_GIT_URL} ${CANOPENNODE_SOURCE_DIR}
            RESULT_VARIABLE CANOPENNODE_FETCH)
        if(NOT CANOPENNODE_FETCH EQUAL 0)
            message(WARNING "CANopen plugin: cannot fetch CANopenNode ${CANOPENNODE_GIT_TAG}; "
                            "built without the CANopen slave")
        endif()
    endif()
endif()
if(EXISTS ${CANOPENNODE_SOURCE_DIR}/CANopen.c)
    file(GLOB CANOPENNODE_301 ${CANOPENNODE_SOURCE_DIR}/301/*.c)
    set(CANOPEN_SLAVE_SOURCES
        ${CANOPENNODE_SOURCE_DIR}/CANopen.c
        ${CANOPENNODE_301}
        ${CMAKE_CURRENT_SOURCE_DIR}/slave/co_driver.c
        ${CMAKE_CURRENT_SOURCE_DIR}/slave/slave_od.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/slave/slave.cpp
    )
    set(CANOPEN_HAVE_SLAVE ON)
    message(STATUS "CANopen plugin: CANopen slave on CANopenNode in ${CANOPENNODE_SOURCE_DIR}")
else()
    set(CANOPEN_SLAVE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/slave/slave_none.cpp)
    set(CANOPEN_HAVE_SLAVE OFF)
endif()

# The plugin's code without the entry points, shared with the tests.
add_library(canopen_core STATIC
    ${CANOPEN_SLAVE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/bus.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/bus_monitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/bus_off_recovery.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/can_link.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/client.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/iec_location.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/link_stall.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/image_io.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/log.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/master_dcf.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/network.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plc_port.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/process_image.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/requests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/status.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/status_server.cpp
    ${OPENPLC_ROOT}/core/src/drivers/plugins/native/cjson/cJSON.c
)
target_include_directories(canopen_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/slave
    ${OPENPLC_ROOT}/core/src/drivers
    ${OPENPLC_ROOT}/core/src/drivers/plugins/native
    ${OPENPLC_ROOT}/core/src/drivers/plugins/native/cjson
    ${OPENPLC_ROOT}/core/src/lib
    ${LELY_INCLUDE_DIR}
)
target_compile_options(canopen_core PRIVATE -Wall -Wextra -Wno-unused-parameter)
if(CANOPEN_HAVE_SLAVE)
    target_include_directories(canopen_core PUBLIC ${CANOPENNODE_SOURCE_DIR})
    # The object types (OD_obj_var_t, ...) are declared only for a dictionary definition, which
    # the slave builds at run time.
    target_compile_definitions(canopen_core PUBLIC OD_DEFINITION)
    # CANopenNode's own sources are compiled as they are, with its warnings left to it.
    set_source_files_properties(${CANOPENNODE_SOURCE_DIR}/CANopen.c ${CANOPENNODE_301}
        PROPERTIES COMPILE_OPTIONS "-w")
endif()
foreach(lib ${LELY_LIBS})
    target_link_libraries(canopen_core PUBLIC ${LELY_LIB_DIR}/liblely-${lib}.a)
endforeach()
target_link_libraries(canopen_core PUBLIC pthread)
if(NOT LELY_PREFIX)
    add_dependencies(canopen_core lely)
endif()

# plugin_image_sizes.c exports set_image_sizes, so the runtime sizes each image table on its own.
add_library(canopen_plugin SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/canopen_plugin.cpp
    ${OPENPLC_ROOT}/core/src/drivers/plugins/native/plugin_image_sizes.c
)
target_compile_options(canopen_plugin PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(canopen_plugin PRIVATE canopen_core)
# Only the entry points are exported, so the static Lely cannot clash with another plugin.
set_target_properties(canopen_plugin PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    LINK_FLAGS "-Wl,--exclude-libs,ALL -Wl,--no-undefined"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
    PREFIX "lib"
    OUTPUT_NAME "canopen_plugin"
    SUFFIX ".so"
)
# The function block library finds the loaded plugin by this name (plc_api.h).
target_link_options(canopen_plugin PRIVATE "-Wl,-soname,libcanopen_plugin.so")
# Never unmapped, so the blocks may keep the table across PLC runs (CANOPEN_PLC_FLAG_PERMANENT).
target_link_options(canopen_plugin PRIVATE "-Wl,-z,nodelete")

# Licence texts of the plugin and of the statically linked Lely, kept with the build.
set(CANOPEN_LICENSE_DIR ${CMAKE_BINARY_DIR}/licenses)
if(LELY_PREFIX)
    set(LELY_LICENSE_DIR ${LELY_PREFIX}/share/licenses/lely-core)
else()
    ExternalProject_Get_Property(lely SOURCE_DIR)
    set(LELY_LICENSE_DIR ${SOURCE_DIR})
endif()
if(CANOPEN_HAVE_SLAVE)
    add_custom_command(TARGET canopen_plugin POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CANOPEN_LICENSE_DIR}/CANopenNode
        COMMAND ${CMAKE_COMMAND} -E copy ${CANOPENNODE_SOURCE_DIR}/LICENSE
            ${CANOPEN_LICENSE_DIR}/CANopenNode/
    )
endif()
add_custom_command(TARGET canopen_plugin POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CANOPEN_LICENSE_DIR}/lely-core
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE ${CANOPEN_LICENSE_DIR}/LICENSE
    COMMAND ${CMAKE_COMMAND} -E copy ${LELY_LICENSE_DIR}/LICENSE ${LELY_LICENSE_DIR}/NOTICE
        ${CANOPEN_LICENSE_DIR}/lely-core/
)

install(TARGETS canopen_plugin
    LIBRARY DESTINATION lib/openplc/plugins
)

if(CANOPEN_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
