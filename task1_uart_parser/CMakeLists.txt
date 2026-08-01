cmake_minimum_required(VERSION 3.16)
project(uart_parser CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

add_library(uart_parser_lib STATIC src/crc16.cpp src/uart_parser.cpp)
target_include_directories(uart_parser_lib PUBLIC include)

add_executable(uart_parser_demo src/main.cpp)
target_link_libraries(uart_parser_demo PRIVATE uart_parser_lib)

# --- Tests (simple, no external framework) ---
enable_testing()
add_executable(uart_parser_tests tests/test_uart_parser.cpp)
target_link_libraries(uart_parser_tests PRIVATE uart_parser_lib)
add_test(NAME uart_parser_tests COMMAND uart_parser_tests)
