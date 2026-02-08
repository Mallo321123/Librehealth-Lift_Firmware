#include "kalman.h"

KalmanFilter::KalmanFilter(const KalmanConfig& config)
{
    Q = config.processNoise;
    R = config.measurementNoise;
    p = config.estimatedError;
    x = config.initialValue;
}

double KalmanFilter::update(double value)
{
    p = p + Q;

    double y = value - x;
    double s = p + R;
    double k = p / s;

    x = x + k * y;
    p = (1 - k) * p;

    return x;
}

double KalmanFilter::current_value()
{
    return x;
}