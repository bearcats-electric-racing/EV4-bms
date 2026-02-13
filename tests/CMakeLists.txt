find_package(CppUTest REQUIRED)

# Manual
# add_executable(tests
#     AllTests.cpp
#     ExampleTest.cpp
# )

# Automatic
file(GLOB_RECURSE TEST_SOURCES
    CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

add_executable(tests
    ${TEST_SOURCES}
)

target_compile_options(tests PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

target_include_directories(tests PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(tests
    CppUTest
    CppUTestExt
)

add_test(NAME all_tests COMMAND tests)