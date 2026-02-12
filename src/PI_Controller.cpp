#include "PI_Controller.h"

static const char *TAG = "PI_Controller";

PI_Controller::PI_Controller(double kp, double ki, double sampleTime, double deadZone, double u_max)
{
    this->setGains(kp, ki);
    this->sampleTime = sampleTime;
    this->deadZone = deadZone;
    this->u_max = u_max;
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

void PI_Controller::reset()
{
    e[0] = 0.0f;
    e[1] = 0.0f;
    u[0] = 0.0f;
    u[1] = 0.0f;
}

float PI_Controller::output(double input, double setPoint)
{
    // -------- PI (Tustin) --------
    e[0] = setPoint - input;
    u[0] = u[1] + e[0]*a + e[1]*b;

    // -------- Dead-zone compensation --------
    float u_dz;
    if(u[0] > 0)
        u_dz = u[0] + deadZone;
    else if(u[0] < 0)
        u_dz = u[0] - deadZone;
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
    if(fabsf(u_sat - u_dz) <= 1e-6)
    {
        e[1] = e[0];
        u[1] = u[0];
    }

    return u_sat;
}


