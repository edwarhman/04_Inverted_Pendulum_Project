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
    car_position_set_pulses(CAR_POSITION_PULSES_PER_METER);
    float result = car_position_get_meters();
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 1.0f, result);
}

TEST_CASE("Pulses to centimeters conversion", "[car]") {
    car_position_set_pulses(CAR_POSITION_PULSES_PER_METER/100);
    float result = car_position_get_centimeters();
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 1.0f, result);
}

TEST_CASE("Pulses to millimeters conversion", "[car]") {
    car_position_set_pulses(CAR_POSITION_PULSES_PER_METER/1000);
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
    car_position_set_pulses(-CAR_POSITION_PULSES_PER_METER);
    float result = car_position_get_meters();
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, -1.0f, result);
}

TEST_CASE("Max range meters", "[car]") {
    car_position_set_max_range_meters(1.5f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 1.5f, car_position_get_max_range_meters());
}

TEST_CASE("Chained add and subtract operations", "[car]") {
    car_position_set_pulses(100);
    car_position_add_pulses(50);
    car_position_add_pulses(25);
    car_position_subtract_pulses(30);
    TEST_ASSERT_EQUAL_INT32(145, car_position_get_pulses());
}

TEST_CASE("Set home resets position to zero", "[car]") {
    car_position_set_pulses(500);
    car_position_add_pulses(200);
    car_position_set_home();
    TEST_ASSERT_EQUAL_INT32(0, car_position_get_pulses());
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, car_position_get_meters());
}

TEST_CASE("Max range zero", "[car]") {
    car_position_set_max_range_meters(0.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, car_position_get_max_range_meters());
}

TEST_CASE("Max range negative value", "[car]") {
    car_position_set_max_range_meters(-0.5f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, -0.5f, car_position_get_max_range_meters());
}

TEST_CASE("Large position values", "[car]") {
    car_position_set_pulses(1000000);
    float result_m = car_position_get_meters();
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 26.88f, result_m);
}

TEST_CASE("Subtract more than current position", "[car]") {
    car_position_set_pulses(50);
    car_position_subtract_pulses(100);
    TEST_ASSERT_EQUAL_INT32(-50, car_position_get_pulses());
}
