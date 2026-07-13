#pragma once

#include "../simulation/Eigen/Dense"
using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::RowVector2d;

class KalmanAltEstimator {
public:
    KalmanAltEstimator(float initial_alt_est);

    /* Set variances associated with input measurements and controls. */
    void setVariances(float alt1_variance, float alt2_variance, float accel_variance);
    /* Update altimeter1 measurement. */
    void updateAltimeter1(float altitude_measurement);
    /* Update altimeter1 measurement. */
    void updateAltimeter2(float altitude_measurement);
    /* Predict current state (altitude and vertical velocity). */
    void predict(float accel_est, float dt);

    /* Column vector where the top row represents altitude and the bottom vertical velocity. */
    Vector2d estimated_state; 

private:
    Matrix2d covariance;
    RowVector2d C;
    float alt1_variance;
    float alt2_variance;
    float accel_variance;
};