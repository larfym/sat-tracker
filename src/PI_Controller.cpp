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
    //Input Low Pass Filter
    i_f = this->alpha * i_f + (1 - this->alpha) * input;

    // Tustin Bilinear
    this->e[0] = setPoint - i_f;
    this->u[0] = this->u[1] + this->e[0]*a + this->e[1]*b;

    // Compensación Zona Muerta
    if(this->u[0] > 0)
    {
        this->u[0] += this->deadZone;
    }else if(this->u[0] < 0)
    {
        this->u[0] -= this->deadZone;
    }else
    {
        this->u[0] = 0;
    }

    // Anti-Windup
    if(abs(this->u[0]) < this->u_max)
    {
        this->e[1] = this->e[0];
        this->u[1] = this->u[0];
    }

    // Saturación
    if(this->u[0] > this->u_max)
    {
        this->u[0] = this->u_max;
    }else if(this->u[0] < -this->u_max)
    {
        this->u[0] = -this->u_max;
    }

    return this->u[0];
}

