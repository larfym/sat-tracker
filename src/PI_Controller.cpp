#include "PI_Controller.h"

static const char *TAG = "PI_Controller";

PI_Controller::PI_Controller(double kp, double ki, double Ts, double u_max, double adm_err)
{
    this->admErr = adm_err;
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

float PI_Controller::output(double value)
{
    this->e[0] = this->SP - value;

    //Tolerable error
    if(abs(this->e[0]) < this->admErr) 
    {
        this->e[0] = 0;
        this->e[1] = 0;
        this->u[1] = 0;
    }

    this->u[0] = this->u[1] + this->e[0]*a + this->e[1]*b;
    this->e[1] = this->e[0];
    this->u[1] = this->u[0];

    //DeadZone
    if(this->u[0] > this->deadZone)
    {
        this->u[0] += this->deadZone;
    }else if(this->u[0] < 0)
    {
        this->u[0] -= this->deadZone;
    }

    //Anti-Windup
    if(this->u[0] > this->u_max)
    {
        this->u[0] = this->u_max;
    }else if(this->u[0] < -this->u_max)
    {
        this->u[0] = -this->u_max;
    } 
    
    return this->u[0];
}

