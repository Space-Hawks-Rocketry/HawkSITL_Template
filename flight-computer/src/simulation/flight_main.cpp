#include "flight_main.hpp"
#include <iostream>

/// Treat these functions like the Arduino setup() and loop() functions. Sensor data
/// is given to you out-of-the-box instead of through your sensor firmware.

const float THRUSTER_MAX_OUTPUT = 15.0; // Newtons
const float CUBE_MASS = 1.0; // kg
const float GRAV_ACCEL = -9.8; // m/s/s
const float TARGET_ALTITUDE = 5; // m 

#pragma once

#include <vector>
#include <numeric>
#include <stdexcept>

class MovingAverage {
public:
    explicit MovingAverage(std::size_t window_size)
        : buffer_(window_size, 0.0),
          window_size_(window_size)
    {
        if (window_size == 0) {
            throw std::invalid_argument("Window size must be greater than zero.");
        }
    }

    /// Add a new sample to the filter.
    void add(double value)
    {
        if (count_ < window_size_) {
            // Buffer isn't full yet.
            buffer_[index_] = value;
            sum_ += value;
            ++count_;
        } else {
            // Replace oldest value.
            sum_ -= buffer_[index_];
            buffer_[index_] = value;
            sum_ += value;
        }

        index_ = (index_ + 1) % window_size_;
    }

    /// Current moving average.
    double average() const
    {
        if (count_ == 0)
            return 0.0;

        return sum_ / static_cast<double>(count_);
    }

    /// Returns true once the full window has been populated.
    bool full() const
    {
        return count_ == window_size_;
    }

    /// Number of samples currently stored.
    std::size_t size() const
    {
        return count_;
    }

    /// Clears all stored samples.
    void reset()
    {
        std::fill(buffer_.begin(), buffer_.end(), 0.0);
        sum_ = 0.0;
        count_ = 0;
        index_ = 0;
    }

private:
    std::vector<double> buffer_;
    std::size_t window_size_;
    std::size_t index_ = 0;
    std::size_t count_ = 0;
    double sum_ = 0.0;
};

// The speed at which this computer will process control steps
float loopFrequencyHz = 100.0;

// Altimeters have low sample rate (5Hz), so must use moving average
// Altimiter makes measurement every 20 computer cycles
MovingAverage altitudeMovingAvg = MovingAverage(60);

// Enable measurement of altitude rate (vertical velocity)
float last_measured_altitude = 0.0;
bool has_measured_altitude = false;

typedef struct CubeStateTag {
    float altitude;
    float altitude_rate;

} CubeState;

std::vector<float> determineThrusterThrottles(CubeState state, std::vector<bool> thruster_status) {
    // Thrusters 1, 2, 3, 4  (must be between 0-1)
    std::vector<float> thruster_throttles = {0.0, 0.0, 0.0, 0.0};
    
    //// Determine throttle needed to hover
    float hover_force = CUBE_MASS * (-GRAV_ACCEL); // force required to hover
    float hover_throttle = (hover_force / THRUSTER_MAX_OUTPUT) / 4; // throttle required for a single thruster to in order to hover
    
    //// PID controller for throttle determination
    float k_P = 0.02;
    float k_D = 0.03;
    float P_error = TARGET_ALTITUDE - state.altitude;
    float D_error = 0.0 - state.altitude_rate;
    // Throttle for a single thruster (assuming all are online). Bias for a hover.
    float single_throttle = k_P * P_error + k_D * D_error + hover_throttle;

    thruster_throttles = {single_throttle,single_throttle,single_throttle,single_throttle};

    return thruster_throttles;
} 

ControlStep setup(json sensor_data) {
    return {.control_msg = {}, .execution_time = 0.0};
}

ControlStep loop(json sensor_data) {
    //// Get sensor values and status
    float altitude1 = sensor_data["altitude1"];
    float altitude2 = sensor_data["altitude2"];
    std::vector<bool> altimeter_status = sensor_data["altimeter_status"];
    std::vector<bool> thruster_status = sensor_data["thruster_status"];

    json control_msg = {}; // Establish empty controls if none are defined.

    //////// BEGIN FLIGHT SOFTWARE

    // Average altimeter readings together for slightly better estimate.
    float altitude;
    if (altimeter_status[0] && altimeter_status[1]) {
        altitude = (altitude1 + altitude2) / 2;
    } else if (altimeter_status[0]) {
        altitude = altitude1;
    } else if (altimeter_status[1]) {
        altitude = altitude2;
    } else {
        // Both altimeters have failed, shit.
    }
    altitudeMovingAvg.add(altitude);
    float altitude_avg = altitudeMovingAvg.average();
    
    //// Determine altitude rate (if possible)
    if (has_measured_altitude) {
        float dt = 1 / loopFrequencyHz;
        float altitude_rate = (altitude_avg - last_measured_altitude) / dt;

        CubeState state = {altitude_avg, altitude_rate};
        std::vector<float> thruster_throttles = determineThrusterThrottles(state, thruster_status);

        //// Construct control msg
        control_msg = {
            {"thruster_throttles", thruster_throttles},
            {"alt_estimate", altitude_avg}
        };
    } else {
        // Cannot measure altitude rate so cannot estimate throttles
        has_measured_altitude = true;
    }
    last_measured_altitude = altitude_avg;

    //////// END FLIGHT SOFTWARE
    
    return {.control_msg = control_msg, .execution_time = 1 / loopFrequencyHz};
}