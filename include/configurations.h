#pragma once
#include <Arduino.h>
#include "driver/gpio.h"

/**
 * @file config.h
 * @brief Centralized configuration file defining pins, communication
 * parameters, timing constants, and control values for the satellite
 * tracking system (Tracker).
 *
 * This file is included across multiple modules and provides:
 * - Pin assignments for motors, sensors, and communication ports.
 * - WiFi, Serial, GPS, and Timer configurations.
 * - PID and system control constants.
 *
 * @note Uses #pragma once to avoid multiple inclusions.
 */

/* -------------------------------------------------------------------------- */
/*                                 PIN DEFINES                                */
/* -------------------------------------------------------------------------- */

/**
 * @name Azimuth Motor Pins (AZ)
 * @brief Control and measurement pins for the Azimuth motor driver.
 */
///@{
#define IN1_AZ_PIN GPIO_NUM_13      /**< Input pin 1 of the Azimuth motor driver. */
#define EN_AZ_PIN GPIO_NUM_12       /**< Enable/PWM pin for Azimuth motor. */
#define IN2_AZ_PIN GPIO_NUM_27      /**< Input pin 2 of the Azimuth motor driver. */

#define I_AZ_PIN GPIO_NUM_39        /**< Analog pin (VN) for Azimuth current sensing. */
#define I_AZ_CHANNEL ADC1_CHANNEL_3 /**< ADC1 channel for Azimuth current sensing. */
///@}

/**
 * @name Elevation Motor Pins (EL)
 * @brief Control and measurement pins for the Elevation motor driver.
 */
///@{
#define IN1_EL_PIN GPIO_NUM_26      /**< Input pin 1 of the Elevation motor driver. */
#define EN_EL_PIN GPIO_NUM_33       /**< Enable/PWM pin for Elevation motor. */
#define IN2_EL_PIN GPIO_NUM_32      /**< Input pin 2 of the Elevation motor driver. */

#define I_EL_PIN GPIO_NUM_36        /**< Analog pin (VP) for Elevation current sensing. */
#define I_EL_CHANNEL ADC1_CHANNEL_0 /**< ADC1 channel for Elevation current sensing. */
///@}

/**
 * @name Communication & Sensor Pins
 * @brief Pins for external modules (GPS) and reed switch.
 */
///@{
#define GPS_RX_PIN 16 /**< GPS UART RX pin. */
#define GPS_TX_PIN 17 /**< GPS UART TX pin. */

#define REED_AZ_PIN GPIO_NUM_19 /**< Reed switch for Azimuth limit detection. */
#define REED_EL_PIN GPIO_NUM_18 /**< Reed switch for Elevation limit detection. */
///@}

/* -------------------------------------------------------------------------- */
/*                          WIFI & SERVER CONFIGURATION                       */
/* -------------------------------------------------------------------------- */

/**
 * @name WiFi Configuration
 * @brief Default parameters for WiFi access point and mDNS service.
 */
///@{
#define DEFAULT_WIFI_SSID "ESP32_AP" /**< Default AP SSID. */
#define DEFAULT_WIFI_PASSWORD ""     /**< Default AP password (empty = open AP). */
#define DEFAULT_SERVER_PORT 80       /**< Web server port. */
#define MDNS_SERVICE_NAME "Tracker"  /**< mDNS hostname. */
#define WIFI_CONNECT_TIMEOUT 10000   /**< WiFi connection timeout in ms. */
///@}

/* -------------------------------------------------------------------------- */
/*                             SERIAL CONFIGURATION                           */
/* -------------------------------------------------------------------------- */

/**
 * @name Serial Debugging
 * @brief Default serial monitor parameters.
 */
///@{
#define SERIAL_BAUDRATE 115200 /**< Baud rate for USB serial monitor. */
///@}

/* -------------------------------------------------------------------------- */
/*                             GPS CONFIGURATION                              */
/* -------------------------------------------------------------------------- */

/**
 * @name GPS Parameters
 * @brief UART settings and default fallback position/time.
 */
///@{
#define GPS_UART 1                  /**< UART index used by GPS. */
#define GPS_BAUDRATE 9600           /**< GPS communication baud rate. */
#define GPS_INTERVAL_SEC 120        /**< GPS update interval in seconds. */

#define DEFAULT_LATITUD 37.7749     /**< Default latitude (fallback). */
#define DEFAULT_LONGITUD -122.4194  /**< Default longitude (fallback). */
#define DEFAULT_ALTITUD 0           /**< Default altitude (meters). */
#define DEFAULT_UNIXTIME 1750015604 /**< Default UNIX timestamp (fallback). */
///@}

/* -------------------------------------------------------------------------- */
/*                           CONTROL & PID CONSTANTS                          */
/* -------------------------------------------------------------------------- */

/**
 * @name Motor Current Limits
 * @brief Safety current limits for motor protection.
 */
///@{
#define CURRENT_AZIMUTH_MAX_mA 1000.0   /**< Max current for Azimuth motor (mA). */
#define CURRENT_ELEVATION_MAX_mA 1000.0 /**< Max current for Elevation motor (mA). */
#define CURRENT_HOMING_mA 10.0          /**< Max current for considering the motor turned off. */
///@}

/**
 * @name Timing Parameters
 * @brief Control-loop timing settings.
 */
///@{
#define TIMER_PREESCALER 80                        /**< Prescaler → 1 MHz timer base (80 MHz / 80). */
#define DEFAULT_SAMPLE_TIME_US 1000000             /**< Default control loop sample time (µs). */
#define SAMPLE_TIME_US 800000                      /**< Control loop sample time (µs). */
#define SAMPLE_TIME_S (SAMPLE_TIME_US / 1000000.0) /**< Control loop sample time (s). */
///@}

/**
 * @name PID Gains — Azimuth
 * @brief Gains for the Azimuth axis PID controller.
 */
///@{
#define KP_AZIMUTH 2.0 /**< Proportional gain (AZ). */
#define KI_AZIMUTH 0.5 /**< Integral gain (AZ). */
#define KD_AZIMUTH 0.1 /**< Derivative gain (AZ). */
///@}

/**
 * @name PID Gains — Elevation
 * @brief Gains for the Elevation axis PID controller.
 */
///@{
#define KP_ELEVATION 2.0 /**< Proportional gain (EL). */
#define KI_ELEVATION 0.5 /**< Integral gain (EL). */
#define KD_ELEVATION 0.1 /**< Derivative gain (EL). */
///@}

/**
 * @name Soft Limit Positions
 * @brief Encoder/step limits for mechanical range enforcement.
 */
///@{
#define REED_AZ_SOFT_LIMIT_HIGH 1900 /**< Upper soft limit for Azimuth. */
#define REED_EL_SOFT_LIMIT_HIGH 500  /**< Upper soft limit for Elevation. */

#define REED_AZ_SOFT_LIMIT_LOW -1    /**< Lower soft limit for Azimuth (-1 = unused). */
#define REED_EL_SOFT_LIMIT_LOW -1    /**< Lower soft limit for Elevation (-1 = unused). */
///@}

/**
 * @brief PWM frequency for PWM motor speed control (Hz).
 */
#define PWM_FREQ_HZ 10000
