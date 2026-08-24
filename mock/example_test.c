// Example test demonstrating how to use the BNO085 SPI Library mocks
// This file is for documentation purposes and is not run by the CI

#include "MockBNO085_SPI_Library.h"
#include "unity.h"

void setUp(void) {
    MockBNO085_SPI_Library_Init();
}

void tearDown(void) {
    MockBNO085_SPI_Library_Verify();
    MockBNO085_SPI_Library_Destroy();
}

void test_example_mock_void_function(void) {
    sensor_meta sensor;
    
    // Expect register_Sensor to be called with specific parameters
    register_Sensor_Expect(&sensor, 1, 0x0001, NULL, 0x0002, NULL, 0x0003, NULL);
    
    // Call the function being tested that should call register_Sensor
    // your_code_that_calls_register_Sensor(&sensor);
    
    // If we called this directly, it would satisfy the expectation:
    // register_Sensor(&sensor, 1, 0x0001, NULL, 0x0002, NULL, 0x0003, NULL);
}

void test_example_mock_function_with_return_value(void) {
    sensor_meta sensor;
    
    // Set up expectation: data_available should return true
    data_available_ExpectAndReturn(&sensor, true);
    
    // Call your code that calls data_available
    // bool result = your_code_that_checks_data_available(&sensor);
    // TEST_ASSERT_TRUE(result);
    
    // Direct call would be:
    // bool result = data_available(&sensor);
    // TEST_ASSERT_TRUE(result);
}

void test_example_mock_function_returning_float(void) {
    sensor_meta sensor;
    
    // Set up multiple expectations
    get_Accelerometer_X_ExpectAndReturn(&sensor, 1.5f);
    get_Accelerometer_Y_ExpectAndReturn(&sensor, 2.5f);
    get_Accelerometer_Z_ExpectAndReturn(&sensor, 3.5f);
    
    // Your code that reads accelerometer values
    // float x = get_Accelerometer_X(&sensor);
    // float y = get_Accelerometer_Y(&sensor);
    // float z = get_Accelerometer_Z(&sensor);
    // TEST_ASSERT_EQUAL_FLOAT(1.5f, x);
    // TEST_ASSERT_EQUAL_FLOAT(2.5f, y);
    // TEST_ASSERT_EQUAL_FLOAT(3.5f, z);
}

void test_example_ignore_function_call(void) {
    sensor_meta sensor;
    
    // Ignore any calls to hardreset_IMU
    hardreset_IMU_Ignore();
    
    // Now hardreset_IMU can be called any number of times
    // your_code_that_might_call_hardreset(&sensor);
}

// Note: This file is not compiled or run automatically.
// It serves as documentation for how to use the mocks in your own tests.
// To use these mocks in your project:
// 1. Add mock/ as a library dependency in your platformio.ini
// 2. Create your test files in your project's test/ directory
// 3. Include "MockBNO085_SPI_Library.h" and "unity.h"
// 4. Write tests following the examples above
