#include "currentSensor.h"

CurrentSensor::CurrentSensor(adc1_channel_t channel)
{
    this->channel = channel;

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(channel, ADC_ATTEN);

    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_NUM_1, ADC_ATTEN, ADC_WIDTH, ADC_DEFAULT_VREF, &adc_chars);
}

uint32_t CurrentSensor::getShuntVoltage_mV()
{
    uint32_t raw = 0;
    for (uint8_t i = 0; i < ADC_MEASURE_SAMPLES; i++)
    {
        raw += adc1_get_raw(channel);
    }
    raw /= ADC_MEASURE_SAMPLES;
    int32_t voltage_mV = esp_adc_cal_raw_to_voltage(raw, &adc_chars) - this->offset_mV;
    if(voltage_mV < 0)
    {
        return 0;
    }
    return voltage_mV;
}

float CurrentSensor::getCurrent_mA()
{
    return this->getShuntVoltage_mV() / (float)(CURRENT_OPAMP_GAIN * CURRENT_SHUNT_VALUE_OHM);
}

void CurrentSensor::calibrateOffset(uint8_t samples)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        uint32_t raw = adc1_get_raw(channel);
        sum += esp_adc_cal_raw_to_voltage(raw, &adc_chars);
    }
    this->offset_mV = sum / samples;
}

uint32_t CurrentSensor::getOffset_mV()
{
    return this->offset_mV;
}