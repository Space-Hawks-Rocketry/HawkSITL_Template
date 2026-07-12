#pragma once

#include "../simulation/Eigen/Dense"
using Eigen::Vector2d;
using Eigen::Matrix2d;

class KalmanAltEstimator {
public:
    KalmanAltEstimator();

    /* Update altimeter1 measurement. */
    void updateAltimeter1(float altitude_measurement);
    /* Update altimeter1 measurement. */
    void updateAltimeter2(float altitude_measurement);
    /* Predict current state (altitude and vertical velocity). */
    void predict(float dt);

    /* Column vector where the top row represents altitude and the bottom vertical velocity. */
    Vector2d estimated_state; 
};