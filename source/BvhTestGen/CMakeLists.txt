cmake_minimum_required(VERSION 3.12)
project(bvh_test_gen LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # to generate compile_commands.json

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#--- GET LOG DIR --------------------------------------------------------
if(NOT DEFINED LOG_DIR)
    set(LOG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/log)
    message(STATUS "Using and creating default LOG_DIR: ${LOG_DIR}")
    file(MAKE_DIRECTORY ${LOG_DIR})
else()
    message(STATUS "Using LOG_DIR from parent: ${LOG_DIR}")
endif()
#------------------------------------------------------------------------

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable (${PROJECT_NAME}
    ${SOURCES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Geometry
    RLogSU
)

target_compile_definitions(${PROJECT_NAME} PRIVATE MODULE_NAME="${PROJECT_NAME}")


if(DEBUG_MODE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_MODE)

    message(STATUS "${PROJECT_NAME}: DEBUG_MODE ENABLED")

else()

    message(STATUS "${PROJECT_NAME}: DEBUG_MODE DISABLED")

endif()
