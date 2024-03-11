project(pidTest)

enable_testing()

add_subdirectory(googletest)

include_directories(${gtest_SOURCE_DIR}/include)
include_directories(${pidLib_SOURCE_DIR})

add_executable(${PROJECT_NAME} pidTest.cpp)

target_link_libraries(${PROJECT_NAME} gtest gtest_main)
target_link_libraries(${PROJECT_NAME} pidLib)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})