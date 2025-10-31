/**
 * @file current.h
 * @brief Definition of the CurrentSensor class for current measurement using the ESP32's ADC.
 *
 * This module allows measuring the current consumed by a motor
 * using the internal Analog-to-Digital Converter (ADC) of the ESP32.
 * The measurement is performed through a **shunt resistor** and, optionally,
 * an **operational amplifier (op-amp)** that conditions the signal.
 *
 * @author Alfonso Mouton
 * @date 2025
 */

#ifndef CURRENT_H
#define CURRENT_H

#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_err.h"

/** @brief ADC attenuation (range of 100 mV to 950 mV). */
#define ADC_ATTEN ADC_ATTEN_DB_0

/** @brief ADC unit used (ADC1). */
#define ADC_NUM_1 ADC_UNIT_1

/** @brief ADC resolution (12 bits = 4096 levels). */
#define ADC_WIDTH ADC_WIDTH_BIT_12

/** @brief Default value of the internal ADC reference (in mV). */
#define ADC_DEFAULT_VREF 1100

/** @brief Shunt resistor value in ohms. */
#define CURRENT_SHUNT_VALUE_OHM 0.22

/** @brief Operational amplifier gain. */
#define CURRENT_OPAMP_GAIN 3

/**
 * @class CurrentSensor
 * @brief Class for current reading using an ESP32 ADC channel.
 *
 * This class configures an ADC channel, calibrates the conversion to millivolts (mV)
 * and calculates the current in milliamperes (mA), considering the shunt resistor
 * and the amplifier gain.
 */
class CurrentSensor
{
private:
    adc1_channel_t channel; 						/**< Configured ADC channel. */
    esp_adc_cal_characteristics_t adc_chars; 	/**< ADC calibration characteristics. */

public:
    /**
     * @brief Constructor for the CurrentSensor class.
     * @param channel The ADC channel to be used for the measurement.
     */
    CurrentSensor(adc1_channel_t channel);

    /**
     * @brief Gets the measured current in milliamperes.
     *
     * This function reads the analog value from the ADC channel, converts it to voltage
     * (mV), and then calculates the equivalent current based on the shunt resistor
     * and the amplifier gain.
     *
     * @return Measured current in milliamperes (mA).
     */
    uint16_t getCurrent_mA();
};

#endif // CURRENT_H