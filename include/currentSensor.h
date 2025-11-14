/**
 * @file current.h
 * @brief Definition of the CurrentSensor class for current measurement using the ESP32's ADC.
 *
 * This module allows measuring the current consumed by a motor
 * using the internal Analog-to-Digital Converter (ADC) of the ESP32.
 * The measurement is performed through a **shunt resistor** and
 * **operational amplifiers (op-amps)** that condition the signal.
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

/** @name ADC Configuration
 * Constants for configuring the ESP32's ADC unit.
 */
///@{
/** @brief ADC attenuation (range of 100 mV to 950 mV) for high precision. */
#define ADC_ATTEN ADC_ATTEN_DB_0

/** @brief ADC unit used (ADC1). */
#define ADC_NUM_1 ADC_UNIT_1

/** @brief ADC resolution (12 bits = 4096 levels). */
#define ADC_WIDTH ADC_WIDTH_BIT_12

/** @brief Default value of the internal ADC reference voltage (in mV). */
#define ADC_DEFAULT_VREF 1100

/** @brief Number of samples to take for averaging during measurement. */
#define ADC_MEASURE_SAMPLES 128
///@}

/** @name Current Measurement Constants
 * Constants specific to the current measurement circuit.
 */
///@{
/** @brief Shunt resistor value in Ohms. */
#define CURRENT_SHUNT_VALUE_OHM 0.22f

/** @brief Operational amplifier gain. */
#define CURRENT_OPAMP_GAIN 2.12f
///@}

/**
 * @class CurrentSensor
 * @brief Class for current reading using an ESP32 ADC channel.
 *
 * This class configures an ADC channel, calibrates the conversion to millivolts (mV),
 * and calculates the current in milliamperes (mA), considering the shunt resistor
 * and the amplifier gain.
 */
class CurrentSensor
{
private:
    adc1_channel_t channel; 			 /**< @brief Configured ADC channel for measurement. */
    esp_adc_cal_characteristics_t adc_chars; /**< @brief ADC calibration characteristics structure. */
    uint32_t offset_mV = 0;			 /**< @brief Voltage offset (in mV) measured under zero current condition. */

public:
    /**
     * @brief Constructor for the CurrentSensor class.
     *
     * Configures the ADC width and attenuation, and performs characterization
     * to obtain the voltage conversion curve.
     *
     * @param channel The ADC1 channel to be used for the measurement.
     */
    CurrentSensor(adc1_channel_t channel);

    /**
     * @brief Gets the measured current in milliamperes (mA).
     *
     * Reads the analog value, applies calibration and offset, and calculates the current
     * using Ohm's Law and the amplifier gain:
     * $I_{mA} = V_{Shunt, mV} / (G \cdot R_{Shunt, \Omega})$
     *
     * @return Measured current in milliamperes (mA). Returns float for precision.
     */
    float getCurrent_mA();

    /**
     * @brief Gets the shunt resistor voltage in millivolts.
     *
     * Takes an average of \ref ADC_MEASURE_SAMPLES raw ADC readings,
     * converts it to voltage using calibration, and subtracts the calibrated offset.
     *
     * @return Shunt voltage in millivolts (mV).
     */
    uint32_t getShuntVoltage_mV();

    /**
     * @brief Calibrates the sensor's voltage offset.
     *
     * Measures the voltage of the ADC pin when the input current is zero (0)
     * to establish the `offset_mV` value. **Must be called with no current applied.**
     *
     * @param samples Number of samples to average for a more stable calibration.
     */
    void calibrateOffset(uint8_t samples);

    /**
     * @brief Gets the calibrated voltage offset value.
     *
     * @return The offset value in millivolts (mV).
     */
    uint32_t getOffset_mV();
};

#endif // CURRENT_H