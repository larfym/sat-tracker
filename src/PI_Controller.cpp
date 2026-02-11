#include "PI_Controller.h"

static const char *TAG = "PI_Controller";

PI_Controller::PI_Controller(double kp, double ki, double sampleTime, double deadZone, double u_max, double inputLowPassFrequency)
{
    this->setGains(kp, ki);
    this->sampleTime = sampleTime;
    this->deadZone = deadZone;
    this->u_max = u_max;
    this->setInputLowPassFilterFrequency(inputLowPassFrequency);
}

void PI_Controller::setGains(double kp, double ki)
{
    this->a = kp + ki*this->sampleTime/2;
    this->b = -kp + ki*this->sampleTime/2;
}

void PI_Controller::setMaxOutput(double u_max)
{
    this->u_max = u_max;
}

void PI_Controller::setDeadZone(double deadZone)
{
    this->deadZone = deadZone;
}

void PI_Controller::setInputLowPassFilterFrequency(double frequency)
{
    this->alpha = exp(-2 * M_PI * frequency * this->sampleTime);
}

float PI_Controller::output(double input, double setPoint)
{
    // -------- Input Low Pass Filter --------
    i_f = alpha * i_f + (1 - alpha) * input;

    // -------- PI (Tustin) --------
    e[0] = setPoint - i_f;
    float u_pi = u[1] + e[0]*a + e[1]*b;

    // -------- Dead-zone compensation --------
    float u_dz;
    if(u_pi > 0)
        u_dz = u_pi + deadZone;
    else if(u_pi < 0)
        u_dz = u_pi - deadZone;
    else
        u_dz = 0;

    // -------- Saturation --------
    float u_sat;
    if(u_dz > u_max)
        u_sat = u_max;
    else if(u_dz < -u_max)
        u_sat = -u_max;
    else
        u_sat = u_dz;

    // -------- Anti-windup --------
    if(fabsf(u_dz) <= u_max)
    {
        e[1] = e[0];
        u[1] = u_pi;
    }

    return u_sat;
}


