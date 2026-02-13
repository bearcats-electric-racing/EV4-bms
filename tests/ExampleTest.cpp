#include <CppUTest/TestHarness.h>

// Check out https://cpputest.github.io/manual.html#test_macros for more

TEST_GROUP(ExampleTests) {
    void setup() {} // Can be used to init things pre tests in the "ExampleTests" test group
    void teardown() {} // Can be used to deinit things post tests in the same test group
};

TEST(ExampleTests, FirstTest) {
    // Compare floats within certain tolerance
    float expected = 14.5f;
    float actual = 14.5f;
    float tolerance = 0.0001;
    DOUBLES_EQUAL(expected, actual, tolerance);
    DOUBLES_EQUAL(25.2f, 25.2f, tolerance);
}

TEST(ExampleTests, SecondTest) {
    // Compare strings
    const char* expected0 = "hello";
    const char* actual0 = "hello";
    STRCMP_EQUAL(expected0, actual0);
    STRCMP_EQUAL("bearcat", "bearcat");
    
    // Compare numbers
    int expected1 = 2;
    int actual1 = 2;
    LONGS_EQUAL(expected1, actual1);
    LONGS_EQUAL(26, 26);

    // Compare any entity using == operator
    bool expected2 = true;
    bool actual2 = true;
    CHECK_EQUAL(expected2, actual2);
}

// To ignore a test (commented out so it doesn't show when tests are run)
// IGNORE_TEST(ExampleTests, ThirdTest) {} 