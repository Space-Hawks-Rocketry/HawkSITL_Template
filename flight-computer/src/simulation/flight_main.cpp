#include "flight_main.hpp"
#include <iostream>

/// Treat these functions like the Arduino setup() and loop() functions. Sensor data
/// is given to you out-of-the-box instead of through your sensor firmware.

const float THRUSTER_MAX_OUTPUT = 15.0; // Newtons
const float CUBE_MASS = 1.0; // kg
const float GRAV_ACCEL = -9.8; // m/s/s

// The speed at which this computer will process control steps
float loopFrequencyHz = 100.0;

ControlStep setup(json sensor_data) {
    return {.control_msg = {}, .execution_time = 0.0};
}

ControlStep loop(json sensor_data) {
    //// Get sensor values and status
    float altitude1 = sensor_data["altitude1"];
    float altitude2 = sensor_data["altitude2"];
    std::vector<bool> altimeter_status = sensor_data["altimeter_status"];
    std::vector<bool> thruster_status = sensor_data["thruster_status"];

    //////// BEGIN FLIGHT SOFTWARE

    // Thrusters 1, 2, 3, 4  (must be between 0-1)
    std::vector<float> thruster_throttles = {0.0, 0.0, 0.0, 0.0};
    
    float hover_force = CUBE_MASS * (-GRAV_ACCEL); // force required to hover
    float hover_throttle = (hover_force / THRUSTER_MAX_OUTPUT) / 4; // throttle required for a single thruster to in order to hover
    
    thruster_throttles = {hover_throttle,hover_throttle,hover_throttle,hover_throttle};

    //////// END FLIGHT SOFTWARE
    
    //// Construct control message
    
    json control_msg = {
        {"thruster_throttles", thruster_throttles}
    };

    return {.control_msg = control_msg, .execution_time = 1 / loopFrequencyHz};
}