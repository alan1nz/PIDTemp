cmake_minimum_required(VERSION 3.7.0)
project(pid VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

add_subdirectory(test)
add_subdirectory(src)

include_directories(src)
# add_executable(pid main.c)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
