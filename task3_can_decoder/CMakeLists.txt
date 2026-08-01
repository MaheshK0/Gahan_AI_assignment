cmake_minimum_required(VERSION 3.16)
project(can_decoder CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

add_library(can_decoder_lib STATIC
    src/json_value.cpp
    src/dbc_database.cpp
    src/signal_codec.cpp
    src/frame_log_reader.cpp
    src/diagnostics.cpp
    src/vehicle_state.cpp
)
target_include_directories(can_decoder_lib PUBLIC include)

add_executable(can_decoder_demo src/main.cpp)
target_link_libraries(can_decoder_demo PRIVATE can_decoder_lib)

enable_testing()
add_executable(can_decoder_tests tests/test_can_decoder.cpp)
target_link_libraries(can_decoder_tests PRIVATE can_decoder_lib)
add_test(NAME can_decoder_tests COMMAND can_decoder_tests)
