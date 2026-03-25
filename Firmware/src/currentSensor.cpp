#include "currentSensor.h"

CurrentSensor::CurrentSensor(adc1_channel_t channel)
{
    this->channel = channel;
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(channel, ADC_ATTEN);
    esp_adc_cal_characterize(ADC_NUM_1, ADC_ATTEN, ADC_WIDTH, ADC_DEFAULT_VREF, &adc_chars);
}

uint32_t CurrentSensor::getShuntVoltage_mV()
{
    int32_t voltage_mV = esp_adc_cal_raw_to_voltage(this->multisample(ADC_MEASURE_SAMPLES), &adc_chars) - this->offset_mV;
    return (voltage_mV < 0)? 0 : voltage_mV;
}

float CurrentSensor::getCurrent_mA()
{
    return this->getShuntVoltage_mV() / (float)(this->opamp_gain * this->shunt_ohm);
}

uint32_t CurrentSensor::getOffset_mV()
{
    return this->offset_mV;
}

void CurrentSensor::setOpAmpGain(float gain)
{
    this->opamp_gain = gain;
}

void CurrentSensor::setShuntValue(float shunt_ohm)
{
    this->shunt_ohm = shunt_ohm;
}

uint32_t CurrentSensor::multisample(uint8_t samples)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        sum += adc1_get_raw(channel);
    }
    return sum / samples;
}

void CurrentSensor::calibrateOffset()
{
    this->offset_mV = esp_adc_cal_raw_to_voltage(this->multisample(ADC_MEASURE_SAMPLES), &adc_chars);
}
