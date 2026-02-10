#include <CppUTest/TestHarness.h>
#include <../src/Utils/TUtils.h>

TEST_GROUP(TUtilsTests) {
    void setup() {}
    void teardown() {}
};

TEST(TUtilsTests, MinMaxShouldReturnAccurateMinAndMax) {
    const float test_data[2][5] = {
        {1.65, 42.0, 55.1, 61.13, -3.13},
        {-743.6, 3.0, 2.77, 2.778, 2.77}
    };
    float expected_min = -743.6;
    float expected_max = 61.13;

    float actual_min;
    float actual_max;
    min_max<2, 5>(test_data, actual_min, actual_max);

    float tolerance = .00001f;
    DOUBLES_EQUAL(expected_min, actual_min, tolerance);
    DOUBLES_EQUAL(expected_max, actual_max, tolerance);
}

TEST(TUtilsTests, MinMaxShouldReturnSameItemForSameItem2DArray) {
    const float test_data[3][3] = {
        {4.5, 4.5, 4.5},
        {4.5, 4.5, 4.5},
        {4.5, 4.5, 4.5}
    };
    float expected_min = 4.5;
    float expected_max = 4.5;

    float actual_min;
    float actual_max;
    min_max<3, 3>(test_data, actual_min, actual_max);

    float tolerance = .00001f;
    DOUBLES_EQUAL(expected_min, actual_min, tolerance);
    DOUBLES_EQUAL(expected_max, actual_max, tolerance);
}

TEST(TUtilsTests, MinMaxShouldReturnSameItemForOneItem2DArray) {
    const float data[1][1] = {{20.0}};
    float expected_min = 20.0;
    float expected_max = 20.0;

    float actual_min;
    float actual_max;
    min_max<1, 1>(data, actual_min, actual_max);

    float tolerance = .00001f;
    DOUBLES_EQUAL(expected_min, actual_min, tolerance);
    DOUBLES_EQUAL(expected_max, actual_max, tolerance);
}

TEST(TUtilsTests, SearchShouldReturnClosestIndexedValueByDefault) {
    const float data[5] = {61.13, 55.1, 42.0, 1.65, -3.13};
    float val = 53.3;
    float expected_index = 1;

    int actual_index = search<5>(data, val);
    LONGS_EQUAL(expected_index, actual_index);

    val = 61.13;
    expected_index = 0;
    actual_index = search<5>(data, val);
    LONGS_EQUAL(expected_index, actual_index);
}

TEST(TUtilsTests, SearchShouldReturnLowerIndexedValueWhenLowerFlagIsSet) {
    const float data[5] = {61.13, 55.1, 42.0, 1.65, -3.13};
    float val = 55.0;
    float expected_index = 2;

    int actual_index = search<5>(data, val, true);
    LONGS_EQUAL(expected_index, actual_index);
}

TEST(TUtilsTests, SearchShouldReturnValidIndexForOutOfBoundsValue) {
    const float data[5] = {61.13, 55.1, 42.0, 1.65, -3.13};
    float val = 73.0;
    float expected_index = 0;

    int actual_index = search<5>(data, val);
    LONGS_EQUAL(expected_index, actual_index);

    val = -35.9954;
    expected_index = 4;
    actual_index = search<5>(data, val);
    LONGS_EQUAL(expected_index, actual_index);
}