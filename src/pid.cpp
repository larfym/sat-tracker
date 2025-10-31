#include "pid.h"

PID::PID(double kp, double ki, double kd)
{
    this->kd = kd;
    this->ki = ki;
    this->kp = kp;
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
    return PID::output(value) * 100 / (min - max);
}