cmake_minimum_required(VERSION 3.17)

project(Triangles LANGUAGES CXX)

find_package(GTest REQUIRED)
enable_testing()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


option(DEBUG_MODE "Enable debug mode" ON)

set(LOG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/log)

add_subdirectory(libs/RLogSU)
add_subdirectory(source/Geometry)


add_executable(gtest_triangles
    source/gtest_triangles.cpp
)

target_link_libraries(gtest_triangles PRIVATE
    RLogSU
    Geometry

    GTest::gtest
    GTest::gtest_main
)


add_executable(test_general
    source/test_general.cpp
)

target_link_libraries(test_general PRIVATE
    RLogSU
    Geometry
)


target_compile_definitions(gtest_triangles PRIVATE MODULE_NAME="gtest_triangles")
target_compile_definitions(test_general    PRIVATE MODULE_NAME="test_general"   )

if(DEBUG_MODE)
    target_compile_definitions(gtest_triangles PRIVATE DEBUG_MODE)
    message(STATUS "gtest_triangles: DEBUG_MODE ENABLED")

    target_compile_definitions(test_general PRIVATE DEBUG_MODE)
    message(STATUS "test_general: DEBUG_MODE ENABLED")

    target_compile_options(test_general PRIVATE
        -g
        -O0
        -fsanitize=address,undefined
        -fno-omit-frame-pointer
    )

    target_link_options(test_general PRIVATE
        -fsanitize=address,undefined
    )

    target_compile_options(gtest_triangles PRIVATE
        -g
        -O0
        -fsanitize=address,undefined
        -fno-omit-frame-pointer
    )
    
    target_link_options(gtest_triangles PRIVATE
        -fsanitize=address,undefined
    )

    message(STATUS "Sanitizers enabled for test targets")

else()
    target_compile_options(test_general
        -O2
    )

    target_compile_options(gtest_triangles
        -O2
    )

    message(STATUS "gtest_triangles: DEBUG_MODE DISABLED")
    message(STATUS "test_general:    DEBUG_MODE DISABLED")
endif()
