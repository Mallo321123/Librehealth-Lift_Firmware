class KalmanFilter
{
    double x, p, Q, R;

private:
    struct KalmanConfig
    {
        double processNoise;
        double measurementNoise;
        double estimatedError;
        double initialValue;
    };

public:
    KalmanFilter(const KalmanConfig& config)
    {
        Q = config.processNoise;
        R = config.measurementNoise;
        p = config.estimatedError;
        x = config.initialValue;
    }

    double update(double value)
    {
        p = p + Q;

        double y = value - x;
        double s = p + R;
        double k = p / s;

        x = x + k * y;
        p = (1 - k) * p;

        return x;
    }

    double current_value()
    {
        return x;
    }

    static constexpr KalmanConfig accel{0.1f, 0.5f, 1.0f, 0.0f};
    static constexpr KalmanConfig gyro{0.1f, 0.5f, 1.0f, 0.0f};
};