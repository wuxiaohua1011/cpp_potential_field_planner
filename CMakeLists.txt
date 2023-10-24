cmake_minimum_required(VERSION 3.5)
project(cpp_potential_field VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(src)
add_library(cpp_potential_field src/potential_field_planning.cpp)

add_executable(main main.cpp)
target_link_libraries(main cpp_potential_field)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
