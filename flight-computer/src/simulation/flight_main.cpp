#include "framework/core/flight_main.hpp"
#include <iostream>
#include <numeric>
#include "simulation/kalman_alt_estimator.hpp"

/// Treat these functions like the Arduino setup() and loop() functions. Sensor data
/// is given to you out-of-the-box instead of through your sensor firmware.

//// Define constants
const float THRUSTER_MAX_OUTPUT = 15.0; // Newtons
const float SUSPENDED_ALTITUDE = 20.0; // The altitude that the cube is initially suspended at (influences initial alt est)
const float CUBE_MASS = 1.0; // kg
const float GRAV_ACCEL = -9.8; // m/s/s
const float TARGET_ALTITUDE = 5; // m 
const float KNOWN_THROTTLE_VARIANCE = 0.0025; // Defined in FlyingCube model by throttle std
// The speed at which this computer will process control steps
const float LOOP_FREQUENCY_HZ = 100.0;
const float LOOP_DT = 1 / LOOP_FREQUENCY_HZ;
// Calculate variance (error) in estimated acceleration (look up error propogation for variance)
float ACCEL_EST_VARIANCE = 4 * powf(THRUSTER_MAX_OUTPUT / CUBE_MASS, 2) * KNOWN_THROTTLE_VARIANCE;

//// Calibration samples and results
float remaining_calibration_time = 5; // seconds
std::vector<float> calib_alt1_samples = {};
std::vector<float> calib_alt2_samples = {};
float alt1_variance = 0;
float alt2_variance = 0;

//// Flight status
enum FlightStatus {
    CALIBRATING,
    READY
};
FlightStatus flight_status = FlightStatus::CALIBRATING;

//// Predicted state
KalmanAltEstimator altitude_estimator(SUSPENDED_ALTITUDE);
float altitude_estimate = SUSPENDED_ALTITUDE;

//// Keep track of thruster throttles (to feed into Kalman altitude estimator)
std::vector<float> thruster_throttles = {0.0, 0.0, 0.0, 0.0};

/* Collect altitude samples during calibration to find variance later. */
void collectCalibrationSamples(json sensor_data) {
    if (sensor_data.contains("altitude1")) {
        calib_alt1_samples.push_back(sensor_data["altitude1"]);
    }
    if (sensor_data.contains("altitude2")) {
        calib_alt2_samples.push_back(sensor_data["altitude2"]);
    }
}

/* Calculate variance from collected calibration samples. */
void calcCalibrationVariances() {
    int alt1_sample_count = calib_alt1_samples.size();
    int alt2_sample_count = calib_alt2_samples.size();
    float alt1_mean = 0;
    float alt2_mean = 0;

    //// Calculate means
    for (float alt_sample : calib_alt1_samples) {
        alt1_mean += alt_sample / alt1_sample_count;
    }
    for (float alt_sample : calib_alt2_samples) {
        alt2_mean += alt_sample / alt2_sample_count;
    }

    //// Calculate variances
    for (float alt_sample : calib_alt1_samples) {
        alt1_variance += powf(alt_sample - alt1_mean, 2) / alt1_sample_count;
    }
    for (float alt_sample : calib_alt2_samples) {
        alt2_variance += powf(alt_sample - alt2_mean, 2) / alt2_sample_count;
    }
}

/* Update altitude_estimate based on available sensor data and thruster throttle. */
void updateAltitudeEstimate(json sensor_data, float total_thruster_throttle) {
    // Calculate controlled acceleration
    float accel_est = GRAV_ACCEL + (THRUSTER_MAX_OUTPUT / CUBE_MASS) * total_thruster_throttle;

    altitude_estimator.predict(accel_est, LOOP_DT);

    if (sensor_data.contains("altitude1")) {
        float altitude_measurement = sensor_data["altitude1"];
        altitude_estimator.updateAltimeter1(altitude_measurement);
    }
    if (sensor_data.contains("altitude2")) {
        float altitude_measurement = sensor_data["altitude2"];
        altitude_estimator.updateAltimeter2(altitude_measurement);
    }

    altitude_estimate = altitude_estimator.estimated_state(0);
}

std::vector<float> determineThrusterThrottles(std::vector<bool> thruster_status) {
    // Thrusters 1, 2, 3, 4  (must be between 0-1)
    std::vector<float> thruster_throttles = {0.0, 0.0, 0.0, 0.0};
    
    //// Determine throttle needed to hover
    float hover_force = CUBE_MASS * (-GRAV_ACCEL); // force required to hover
    float hover_throttle = (hover_force / THRUSTER_MAX_OUTPUT) / 4; // throttle required for a single thruster to in order to hover
    
    //// PID controller for throttle determination
    float k_P = 0.02;
    float k_D = 0.04;
    float P_error = TARGET_ALTITUDE - altitude_estimator.estimated_state[0];
    float D_error = 0.0 - altitude_estimator.estimated_state[1];
    // Throttle for a single thruster (assuming all are online). Bias for a hover.
    float single_throttle = k_P * P_error + k_D * D_error + hover_throttle;

    thruster_throttles = {single_throttle,single_throttle,single_throttle,single_throttle};

    //// Handle bad thrusters
    if (!thruster_status[0] || !thruster_status[2]) {
        thruster_throttles = {0, 2*single_throttle, 0, 2*single_throttle};
    } else if (!thruster_status[1] || !thruster_status[3]) {
        thruster_throttles = {2*single_throttle, 0, 2*single_throttle, 0};
    }

    return thruster_throttles;
} 


ControlStep setup(json sensor_data) {
    return {.control_msg = {}, .execution_time = 0.0};
}

ControlStep loop(json sensor_data) {
    json control_msg;

    switch (flight_status)
    {
    case FlightStatus::CALIBRATING:
        if (remaining_calibration_time > 0) {
            // still collecting calibration samples
            collectCalibrationSamples(sensor_data);
        } else{
            // finished collecting calibration samples
            calcCalibrationVariances();
            altitude_estimator.setVariances(alt1_variance, alt2_variance, ACCEL_EST_VARIANCE);

            //// Log calculated variances
            std::cout << "---------- Calibration Results ----------" << std::endl;
            std::cout << "Altimeter 1 Standard Dev. (m): " << sqrt(alt1_variance) << std::endl;
            std::cout << "Altimeter 2 Standard Dev. (m): " << sqrt(alt2_variance) << std::endl;
            std::cout << "Accel. Estimate Standard Dev. (m/s^2): " << sqrt(ACCEL_EST_VARIANCE) << std::endl << std::endl;

            flight_status = FlightStatus::READY;
            control_msg["flight_status"] = "READY";
        }
        remaining_calibration_time -= LOOP_DT;
        break;
    case FlightStatus::READY:
        float total_thruster_throttle = thruster_throttles[0] + thruster_throttles[1] + thruster_throttles[2] + thruster_throttles[3];        
        updateAltitudeEstimate(sensor_data, total_thruster_throttle);
        control_msg["alt_estimate"] = altitude_estimate;
        std::vector<bool> thruster_status = sensor_data["thruster_status"];
        thruster_throttles = determineThrusterThrottles(thruster_status);
        control_msg["thruster_throttles"] = thruster_throttles;
        break;
    }

    return {.control_msg = control_msg, .execution_time = LOOP_DT};
}



// // Enable measurement of altitude rate (vertical velocity)
// float last_measured_altitude = 0.0;
// bool has_measured_altitude = false;

// typedef struct CubeStateTag {
//     float altitude;
//     float altitude_rate;

// } CubeState;

// std::vector<float> determineThrusterThrottles(CubeState state, std::vector<bool> thruster_status) {
//     // Thrusters 1, 2, 3, 4  (must be between 0-1)
//     std::vector<float> thruster_throttles = {0.0, 0.0, 0.0, 0.0};
    
//     //// Determine throttle needed to hover
//     float hover_force = CUBE_MASS * (-GRAV_ACCEL); // force required to hover
//     float hover_throttle = (hover_force / THRUSTER_MAX_OUTPUT) / 4; // throttle required for a single thruster to in order to hover
    
//     //// PID controller for throttle determination
//     float k_P = 0.02;
//     float k_D = 0.03;
//     float P_error = TARGET_ALTITUDE - state.altitude;
//     float D_error = 0.0 - state.altitude_rate;
//     // Throttle for a single thruster (assuming all are online). Bias for a hover.
//     float single_throttle = k_P * P_error + k_D * D_error + hover_throttle;

//     thruster_throttles = {single_throttle,single_throttle,single_throttle,single_throttle};

//     return thruster_throttles;
// } 

// ControlStep setup(json sensor_data) {
//     return {.control_msg = {}, .execution_time = 0.0};
// }

// ControlStep loop(json sensor_data) {
//     //// Get sensor values and status
//     float altitude1 = sensor_data["altitude1"];
//     float altitude2 = sensor_data["altitude2"];
//     std::vector<bool> altimeter_status = sensor_data["altimeter_status"];
//     std::vector<bool> thruster_status = sensor_data["thruster_status"];

//     json control_msg = {}; // Establish empty controls if none are defined.

//     //////// BEGIN FLIGHT SOFTWARE

//     // Average altimeter readings together for slightly better estimate.
//     float altitude;
//     if (altimeter_status[0] && altimeter_status[1]) {
//         altitude = (altitude1 + altitude2) / 2;
//     } else if (altimeter_status[0]) {
//         altitude = altitude1;
//     } else if (altimeter_status[1]) {
//         altitude = altitude2;
//     } else {
//         // Both altimeters have failed, shit.
//     }
//     altitudeMovingAvg.add(altitude);
//     float altitude_avg = altitudeMovingAvg.average();
    
//     //// Determine altitude rate (if possible)
//     if (has_measured_altitude) {
//         float dt = 1 / LOOP_FREQUENCY_HZ;
//         float altitude_rate = (altitude_avg - last_measured_altitude) / dt;

//         CubeState state = {altitude_avg, altitude_rate};
//         std::vector<float> thruster_throttles = determineThrusterThrottles(state, thruster_status);

//         //// Construct control msg
//         control_msg = {
//             {"thruster_throttles", thruster_throttles},
//             {"alt_estimate", altitude_avg}
//         };
//     } else {
//         // Cannot measure altitude rate so cannot estimate throttles
//         has_measured_altitude = true;
//     }
//     last_measured_altitude = altitude_avg;

//     //////// END FLIGHT SOFTWARE
    
//     return {.control_msg = control_msg, .execution_time = 1 / LOOP_FREQUENCY_HZ};
// }