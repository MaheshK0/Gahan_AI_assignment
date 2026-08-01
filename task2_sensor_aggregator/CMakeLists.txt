cmake_minimum_required(VERSION 3.16)
project(sensor_aggregator CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Threads REQUIRED)

add_library(sensor_aggregator_lib STATIC
    src/csv_reader.cpp
    src/vehicle_state.cpp
    src/sensor_producer.cpp
)
target_include_directories(sensor_aggregator_lib PUBLIC include)
target_link_libraries(sensor_aggregator_lib PUBLIC Threads::Threads)

add_executable(sensor_aggregator_demo src/main.cpp)
target_link_libraries(sensor_aggregator_demo PRIVATE sensor_aggregator_lib)
