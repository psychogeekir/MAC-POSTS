#cmake_minimum_required(VERSION 2.8.12)

add_subdirectory(pybind11)

file(GLOB_RECURSE minamiapi_src "src/*.cpp" "src/*.h")
# define the output python library
pybind11_add_module(MNMAPI ${minamiapi_src})
target_link_libraries (MNMAPI PRIVATE Snap minami Eigen3::Eigen)
