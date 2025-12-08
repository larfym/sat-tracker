#include "PI_Controller.h"

static const char *TAG = "PI_Controller";

PI_Controller::PI_Controller(double kp, double ki, double Ts, double u_max)
{
    this->setGains(kp, ki, Ts);
    this->u_max = u_max;
}

void PI_Controller::setGains(double kp, double ki, double Ts)
{
    this->a = kp + ki*Ts/2;
    this->b = -kp + ki*Ts/2;
}

void PI_Controller::setMaxOutput(double u_max)
{
    this->u_max = u_max;
}

void PI_Controller::setPoint(double SP)
{
    this->SP = SP;
}

void PI_Controller::setDeadZone(double deadZone)
{
    this->deadZone = deadZone;
}

double PI_Controller::output(double value)
{
    e[0] = this->SP - value;

    //Tolerable error
    if(abs(e[0]) < 0.6) 
    {
        e[0] = 0;
        e[1] = 0;
    }

    u[0] = u[1] + this->a*e[0] + this->b*e[1];
    e[1] = e[0];
    u[1] = u[0];

    //DeadZone
    if(u[0] > this->deadZone)
    {
        u[0] += this->deadZone;
    }else if(u[0] < 0)
    {
        u[0] -= this->deadZone;
    }

    //Anti-Windup
    if(u[0] > this->u_max)
    {
        u[0] = this->u_max;
    }else if(u[0] < -this->u_max)
    {
        u[0] = -this->u_max;
    } 
    
    return u[0];
}

