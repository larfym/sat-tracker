#include <driver/timer.h>
#include <esp_log.h>

class PID
{
private:
    double kp, ki, kd, SP;
    double prev_error, integral;
    double min_I, max_I;

public:
    PID(double kp, double ki, double kd);
    double output(double value);
    double outputAsPercent(double value, double min, double max);
    void setConstraintsAtIntegralError(double min, double max);
    void setPoint(double sp);
    void setGains(double kp, double ki, double kd);
};
