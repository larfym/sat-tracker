#include <driver/timer.h>
#include <esp_log.h>
#include <math.h>

class PI_Controller
{
private:
    double a, b, sampleTime, deadZone = 0.0, u_max;
    double i_f = 0.0;
    double e[2] = {0.0,0.0};
    double u[2] = {0.0,0.0};

public:
    PI_Controller(double kp, double ki, double sampleTime, double deadZone, double u_max);
    void setGains(double kp, double ki);
    void setMaxOutput(double u_max);
    void setDeadZone(double deadZone);
    void reset();
    float output(float input, float setPoint);
};
