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
    this->a = kp + ki*this->sampleTime;
    this->b = -kp;
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

float PI_Controller::output(float input, float setPoint)
{
    // -------- PI (Backward Euler) --------
    e[0] = setPoint - input;
    u[0] = u[1] + e[0]*a + e[1]*b;

    // -------- Dead-zone compensation --------
    float u_dz = u[0];
    if(u[0] > 0)
        u_dz += deadZone;
    else if(u[0] < 0)
        u_dz -= deadZone;

    // -------- Saturation --------
    float u_sat = fminf(fmaxf(u_dz, -u_max), u_max);

    // -------- Anti-windup --------
    if(u_dz == u_sat)   // Only update the integral term if the output is not saturated
    {
        u[1] = u[0];
        e[1] = e[0];
    }

    return u_sat;
}


