#ifndef KALMAN_H
#define KALMAN_H

class KalmanFilter
{
    double x, p, Q, R;

public:
    struct KalmanConfig
    {
        double processNoise;
        double measurementNoise;
        double estimatedError;
        double initialValue;
    };

    KalmanFilter(const KalmanConfig& config);

    double update(double value);
    double current_value();

    inline static constexpr KalmanConfig accel{0.1, 0.5, 1.0, 0.0};
    inline static constexpr KalmanConfig gyro{0.1, 0.5, 1.0, 0.0};
    inline static constexpr KalmanConfig pressure{0.1, 0.5, 1.0, 0.0};
};

#endif // KALMAN_H
