cmake_minimum_required(VERSION 3.10)

project(test_bvh)
add_subdirectory(bvh)
add_executable(Tutorial main.cpp)
target_link_libraries(Tutorial PUBLIC bvh)



