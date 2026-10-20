cmake_minimum_required(VERSION 3.20)
project(ssm2045_chip LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ssm2045_chip STATIC src/chip.cpp)
target_include_directories(ssm2045_chip PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(ssm2045_chip PRIVATE -Wall -Wextra)

add_executable(ssm2045_chip_tests tests/chip_tests.cpp)
target_link_libraries(ssm2045_chip_tests PRIVATE ssm2045_chip)
target_compile_options(ssm2045_chip_tests PRIVATE -Wall -Wextra)

add_executable(ssm2045_chip_render examples/render.cpp)
target_link_libraries(ssm2045_chip_render PRIVATE ssm2045_chip)
target_compile_options(ssm2045_chip_render PRIVATE -Wall -Wextra)

add_executable(ssm2045_chip_demo examples/chip_demo.cpp)
target_link_libraries(ssm2045_chip_demo PRIVATE ssm2045_chip)
target_compile_options(ssm2045_chip_demo PRIVATE -Wall -Wextra)

install(TARGETS ssm2045_chip ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

enable_testing()
add_test(NAME ssm2045_chip_tests COMMAND ssm2045_chip_tests)
