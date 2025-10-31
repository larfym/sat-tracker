#include "currentMeasure.h"

CurrentSensor::CurrentSensor(adc1_channel_t channel)
{
    this->channel = channel;

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(channel, ADC_ATTEN);

    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_NUM_1, ADC_ATTEN, ADC_WIDTH, ADC_DEFAULT_VREF, &adc_chars);   
}

uint16_t CurrentSensor::getCurrent_mA()
{
    uint32_t raw = 0;
    for (uint8_t i = 0; i < 64; i++)
    {
        raw += adc1_get_raw(channel);
    }
    raw /= 64;
    return esp_adc_cal_raw_to_voltage(raw, &adc_chars)/(CURRENT_OPAMP_GAIN*CURRENT_SHUNT_VALUE_OHM);
}