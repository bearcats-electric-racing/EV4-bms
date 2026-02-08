find_package(CppUTest REQUIRED)

add_executable(tests
    AllTests.cpp
    Example/ExampleTest.cpp
)

target_include_directories(tests PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(tests
    CppUTest
    CppUTestExt
)

add_test(NAME all_tests COMMAND tests)