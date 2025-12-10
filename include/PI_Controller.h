#include <driver/timer.h>
#include <esp_log.h>

class PI_Controller
{
private:
    double a, b, SP, u_max, deadZone = 0.0, admErr = 0.0;
    double e[2] = {0.0,0.0};
    double u[2] = {0.0,0.0};

public:
    PI_Controller(double kp, double ki, double Ts, double u_max, double adm_err);
    void setGains(double kp, double ki, double sampleTime);
    void setMaxOutput(double u_max);
    void setDeadZone(double deadZone);
    void setPoint(double SP);
    float output(double value);
};
