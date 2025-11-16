#include "pid.h"

static const char *TAG = "PID";

PID::PID(double kp, double ki, double kd)
{
    this->setGains(kp, ki, kd);
    this->SP = 0.0;
    this->integral = 0.0;

    this->min_I = -1e9;
    this->max_I = 1e9;
}

void PID::setConstraintsAtIntegralError(double min, double max)
{
    this->min_I = min;
    this->max_I = max;
}

void PID::setGains(double kp, double ki, double kd)
{
    this->kd = kd;
    this->ki = ki;
    this->kp = kp;
}

void PID::setPoint(double sp)
{
    this->SP = sp;
}

double PID::output(double value)
{
    double error = this->SP - value;
    this->integral += error;

    if (integral > max_I)
    {
        integral = max_I;
    }
    else if (integral < min_I)
    {
        integral = min_I;
    }

    double derivative = error - prev_error;

    return (double)(kp * error + this->ki * integral + kd * derivative);
}

double PID::outputAsPercent(double value, double min, double max)
{
    double out = PID::output(value);
    if(min == max) return 0.0;

    double normalized = (out - min) / (max - min);

    return normalized*100.0;
}