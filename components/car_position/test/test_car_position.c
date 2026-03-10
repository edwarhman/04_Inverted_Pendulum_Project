#include <car_position.h>
#include <unity.h>

#define EPSILON 0.006f

TEST_CASE("Zero position at start", "[car]") {
    car_position_reset();
    TEST_ASSERT_EQUAL_INT32(0, car_position_get_pulses());
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, car_position_get_meters());
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, car_position_get_centimeters());
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, car_position_get_millimeters());
}

TEST_CASE("Pulses to meters conversion", "[car]") {
    car_position_set_pulses(37200);
    float result = car_position_get_meters();
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 1.0f, result);
}

TEST_CASE("Pulses to centimeters conversion", "[car]") {
    car_position_set_pulses(372);
    float result = car_position_get_centimeters();
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 1.0f, result);
}

TEST_CASE("Pulses to millimeters conversion", "[car]") {
    car_position_set_pulses(37);
    float result = car_position_get_millimeters();
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 1.0f, result);
}

TEST_CASE("Add pulses", "[car]") {
    car_position_set_pulses(100);
    car_position_add_pulses(50);
    TEST_ASSERT_EQUAL_INT32(150, car_position_get_pulses());
}

TEST_CASE("Subtract pulses", "[car]") {
    car_position_set_pulses(100);
    car_position_subtract_pulses(30);
    TEST_ASSERT_EQUAL_INT32(70, car_position_get_pulses());
}

TEST_CASE("Reset position", "[car]") {
    car_position_set_pulses(1000);
    car_position_reset();
    TEST_ASSERT_EQUAL_INT32(0, car_position_get_pulses());
}

TEST_CASE("Negative pulses", "[car]") {
    car_position_set_pulses(-37200);
    float result = car_position_get_meters();
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, -1.0f, result);
}

TEST_CASE("Range meters", "[car]") {
    car_position_set_range_meters(1.5f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 1.5f, car_position_get_range_meters());
}
