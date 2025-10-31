#include "reed.h"

ReedSwitch::ReedSwitch(gpio_num_t pin, pcnt_unit_t unit, pcnt_channel_t channel, uint16_t l_lim, uint16_t h_lim, void (*eventHandler)(void *))
{
    this->pin = pin;
    this->unit = unit;
    this->channel = channel;
    this->current_dir = FORWARD;

    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);

    pcnt_config_t pcnt;
    pcnt.channel = channel;
    pcnt.pulse_gpio_num = pin;
    pcnt.ctrl_gpio_num = -1;
    pcnt.lctrl_mode = PCNT_MODE_KEEP;
    pcnt.hctrl_mode = PCNT_MODE_KEEP;
    pcnt.pos_mode = PCNT_COUNT_DIS;
    pcnt.neg_mode = PCNT_COUNT_INC;
    pcnt.counter_h_lim = h_lim;
    pcnt.counter_l_lim = l_lim;
    pcnt.unit = unit;

    pcnt_unit_config(&pcnt);
    pcnt_set_filter_value(unit, REED_DEBOUNCE);
    pcnt_filter_enable(unit);

    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);
    pcnt_intr_enable(unit);

    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(unit, eventHandler, NULL);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
}

int16_t ReedSwitch::getCount()
{
    int16_t count;
    pcnt_get_counter_value(this->unit, &count);
    return count;
}

void ReedSwitch::stopCount()
{
    pcnt_counter_pause(this->unit);
}

void ReedSwitch::startCount()
{
    pcnt_counter_resume(this->unit);
}

void ReedSwitch::resetCount()
{
    pcnt_counter_pause(this->unit);
    pcnt_counter_clear(this->unit);
}

void ReedSwitch::countDirection(direction dir)
{
    if (dir == FORWARD)
    {
        this->current_dir = FORWARD;
        pcnt_set_mode(this->unit, this->channel, PCNT_COUNT_DIS, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
    }
    else if (dir == BACKWARD)
    {
        this->current_dir = BACKWARD;
        pcnt_set_mode(this->unit, this->channel, PCNT_COUNT_DIS, PCNT_COUNT_DEC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
    }
}

pcnt_unit_t ReedSwitch::getUnit()
{
    return this->unit;
}