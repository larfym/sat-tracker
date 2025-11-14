#include "motorDriver.h"

uint8_t MotorDriver::id = 0;

static const char *TAG = "MOTOR";

MotorDriver::MotorDriver(gpio_num_t IN1_pin, gpio_num_t IN2_pin, gpio_num_t PWM_pin, ledc_timer_t timer, ledc_channel_t channel, uint16_t frequency)
{
    this->IN1_pin = IN1_pin;
    this->IN2_pin = IN2_pin;
    this->channel = channel;
    this->current_direction = FORWARD;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << IN1_pin) | (1ULL << IN2_pin) | (1ULL << PWM_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(IN1_pin, 0);
    gpio_set_level(IN2_pin, 0);

    ledc_timer_config_t pwm_tim = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT, // Rango 0-1023
        .timer_num = timer,
        .freq_hz = frequency,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&pwm_tim);

    ledc_channel_config_t pwm_ch = {
        .gpio_num = PWM_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&pwm_ch);

    MotorDriver::id++;
    this->instance_id = MotorDriver::id;
    ESP_LOGI(TAG, "Motor %d Config Done", instance_id);
}

void MotorDriver::setDuty(double percentage)
{
    if (percentage < 0.0)
        percentage = 0.0;
    if (percentage > 100.0)
        percentage = 100.0;

    uint32_t duty_val = (uint32_t)((percentage / 100.0) * 1023);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty_val);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void MotorDriver::setDirection(direction dir)
{
    if (dir == FORWARD)
    {
        gpio_set_level(IN1_pin, 1);
        gpio_set_level(IN2_pin, 0);
        this->current_direction = FORWARD;
    }
    else if (dir == BACKWARD)
    {
        gpio_set_level(IN1_pin, 0);
        gpio_set_level(IN2_pin, 1);
        this->current_direction = BACKWARD;
    }

    ESP_LOGD(TAG, "Motor %d direction set to: %s", instance_id, (this->current_direction == FORWARD ? "FORWARD" : "BACKWARD"));
}

void MotorDriver::toggleDirection()
{
    direction new_dir = (this->current_direction == FORWARD) ? BACKWARD : FORWARD;
    setDirection(new_dir);
    ESP_LOGD(TAG, "Motor %d direction set to: %s", instance_id, (this->current_direction == FORWARD ? "FORWARD" : "BACKWARD"));
}

void MotorDriver::stop()
{
    gpio_set_level(IN1_pin, 0);
    gpio_set_level(IN2_pin, 0);

    ESP_LOGD(TAG, "Motor %d stopped", instance_id);
}

void MotorDriver::setMotorLowerBound(double percentage)
{
    this->lower_bound_percentage = percentage;
}


void MotorDriver::setMotorDuty(double percentage)
{
    if(percentage == lower_bound_percentage)
    {
        this->setDuty(0.0);
    }else
    {
        percentage = ((100.0 - lower_bound_percentage) * (percentage / 100.0)) + lower_bound_percentage;
        this->setDuty(percentage);
    }
}