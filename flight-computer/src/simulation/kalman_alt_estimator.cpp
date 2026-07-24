#include "simulation/kalman_alt_estimator.hpp"

KalmanAltEstimator::KalmanAltEstimator(float initial_alt_est=0) {   
    estimated_state << initial_alt_est, 0.0; 
    covariance << 100.0, 0.0,
                  0.0, 100.0;
    C << 1.0, 0.0;
}

/* Set variances associated with input measurements and controls. */
void KalmanAltEstimator::setVariances(float alt1_variance, float alt2_variance, float accel_variance) {
    this->alt1_variance = alt1_variance;
    this->alt2_variance = alt2_variance;
    this->accel_variance = accel_variance;
}

/* Update altimeter1 measurement. */
void KalmanAltEstimator::updateAltimeter1(float altitude_measurement) {
    Vector2d K = covariance * C.transpose() * (1 / (C * covariance * C.transpose() + alt1_variance));
    estimated_state = estimated_state + K * (altitude_measurement - C * estimated_state);
    covariance = (Matrix2d::Identity() - K * C) * covariance;
}

/* Update altimeter1 measurement. */
void KalmanAltEstimator::updateAltimeter2(float altitude_measurement) {
    Vector2d K = covariance * C.transpose() * (1 / (C * covariance * C.transpose() + alt2_variance));
    estimated_state = estimated_state + K * (altitude_measurement - C * estimated_state);
    covariance = (Matrix2d::Identity() - K * C) * covariance;
}

/* Predict current state (altitude and vertical velocity). Must be done before updating. */
void KalmanAltEstimator::predict(float accel_est, float dt) {
    Matrix2d A;
    A << 1.0, dt,
         0, 1;

    Vector2d B;
    B << dt*dt / 2, dt;

    Matrix2d R = B * accel_variance * B.transpose();

    estimated_state = A * estimated_state + B * accel_est;
    covariance = A * covariance * A.transpose() + R;
}