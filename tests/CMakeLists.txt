find_path(CPPUTEST_INCLUDE_DIR
    NAMES CppUTest/TestHarness.h
)

find_library(CPPUTEST_LIBRARY
    NAMES CppUTest
)

find_library(CPPUTEST_EXT_LIBRARY
    NAMES CppUTestExt
)

if (NOT CPPUTEST_INCLUDE_DIR OR NOT CPPUTEST_LIBRARY)
    message(FATAL_ERROR "CppUTest not found")
endif()

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
    ${CPPUTEST_INCLUDE_DIR}
)

target_link_libraries(tests
    ${CPPUTEST_LIBRARY}
    ${CPPUTEST_EXT_LIBRARY}
)

add_test(NAME all_tests COMMAND tests)