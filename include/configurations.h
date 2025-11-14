#pragma once
#include <Arduino.h>
#include "driver/gpio.h"

/**
 * @file config.h
 * @brief Pin definitions, communication configurations (WiFi, GPS, Serial),
 * and control parameters for the tracking system (Tracker).
 *
 * @note This file uses the #pragma once directive to ensure it is included only once.
 */

/* PIN DEFINITIONS ------------------------------------------------------------- */

/** @name Azimuth Motor Pins (AZ)
 * Definitions for the control and measurement pins of the Azimuth motor.
 */
///@{
#define IN1_AZ_PIN GPIO_NUM_13  /**< @brief Input pin 1 of the Azimuth motor driver. */
#define EN_AZ_PIN GPIO_NUM_12   /**< @brief Enable (PWM) pin of the Azimuth motor driver. */
#define IN2_AZ_PIN GPIO_NUM_27  /**< @brief Input pin 2 of the Azimuth motor driver. */
#define I_AZ_PIN GPIO_NUM_39    /**< @brief Analog input pin (VN) for current measurement of the Azimuth motor. */
#define I_AZ_CHANNEL ADC1_CHANNEL_3 /**< @brief ADC1 channel corresponding to the Azimuth current measurement pin. */
///@}

/** @name Elevation Motor Pins (EL)
 * Definitions for the control and measurement pins of the Elevation motor.
 */
///@{
#define IN1_EL_PIN GPIO_NUM_26  /**< @brief Input pin 1 of the Elevation motor driver. */
#define EN_EL_PIN GPIO_NUM_33   /**< @brief Enable (PWM) pin of the Elevation motor driver. */
#define IN2_EL_PIN GPIO_NUM_32  /**< @brief Input pin 2 of the Elevation motor driver. */
#define I_EL_PIN GPIO_NUM_36    /**< @brief Analog input pin (VP) for current measurement of the Elevation motor. */
#define I_EL_CHANNEL ADC1_CHANNEL_0 /**< @brief ADC1 channel corresponding to the Elevation current measurement pin. */
///@}

/** @name Communication and Sensor Pins
 * Definitions for external modules and limit switches.
 */
///@{
#define GPS_RX_PIN 16           /**< @brief RX (Receiver) pin for Serial communication with the GPS module. */
#define GPS_TX_PIN 17           /**< @brief TX (Transmitter) pin for Serial communication with the GPS module. */

#define REED_AZ_PIN GPIO_NUM_19 /**< @brief Pin for the Azimuth magnetic limit sensor (Reed switch). */
#define REED_EL_PIN GPIO_NUM_18 /**< @brief Pin for the Elevation magnetic limit sensor (Reed switch). */
///@}

/* WIFI CONFIGURATIONS --------------------------------------------------------- */

/** @name WiFi and Server Configurations
 * Network and web server parameters.
 */
///@{
#define DEFAULT_WIFI_SSID "ESP32_AP" /**< @brief Default WiFi network name (SSID). */
#define DEFAULT_WIFI_PASSWORD ""     /**< @brief Default WiFi network password (empty). */
#define DEFAULT_SERVER_PORT 80       /**< @brief Default port for the web server. */
#define MDNS_SERVICE_NAME "Tracker"  /**< @brief mDNS (Multicast DNS) service name. */
#define WIFI_CONNECT_TIMEOUT 10000   /**< @brief Maximum waiting time for WiFi connection (in ms). */
///@}

/* SERIAL CONFIGURATIONS ------------------------------------------------------- */

/** @name Serial Configurations
 * Parameters for serial debugging communication.
 */
///@{
#define SERIAL_BAUDRATE 115200  /**< @brief Baud rate for serial (USB/monitor) communication. */
///@}

/* TIMER CONFIGURATIONS -------------------------------------------------------- */

/** @name Timer Configurations
 * Parameters for internal timer setup.
 */
///@{
#define SECOND_TIMER_PREESCALER 80      /**< @brief Timer prescaler to achieve a 1 microsecond time base (80MHz / 80 = 1MHz). */
#define SECOND_TIMER_ALARM_VALUE 1000000 /**< @brief Timer alarm value for a 1-second period (1,000,000 microseconds). */
///@}

/* GPS CONFIGURATIONS ---------------------------------------------------------- */

/** @name GPS Configurations
 * Communication parameters and default location values.
 */
///@{
#define GPS_UART 1             /**< @brief UART number used for GPS communication. */
#define GPS_BAUDRATE 9600      /**< @brief Baud rate for communication with the GPS module. */
#define GPS_INTERVAL_SEC 120   /**< @brief Time interval (in seconds) to update the GPS position. */
#define DEFAULT_LATITUD 37.7749 /**< @brief Default latitude (e.g., San Francisco, CA). */
#define DEFAULT_LONGITUD -122.4194 /**< @brief Default longitude (e.g., San Francisco, CA). */
#define DEFAULT_ALTITUD 0        /**< @brief Default altitude (in meters). */
#define DEFAULT_UNIXTIME 1750015604 /**< @brief Default UNIX time (e.g., June 12, 2025). */
///@}

/* CONTROL CONFIGURATIONS ------------------------------------------------------ */

/** @name System Control Configurations
 * General operating parameters and limits.
 */
///@{
#define PWM_FREQ 10000          /**< @brief PWM frequency used for motor speed control (in Hz). */
#define REED_AZ_SOFT_LIMIT_HIGH 1900 /**< @brief Upper soft limit position for Azimuth (in encoder/step units). */
#define REED_EL_SOFT_LIMIT_HIGH 500  /**< @brief Upper soft limit position for Elevation (in encoder/step units). */
#define REED_AZ_SOFT_LIMIT_LOW -1   /**< @brief Lower soft limit position for Azimuth (value -1 indicates unused or physical limit). */
#define REED_EL_SOFT_LIMIT_LOW -1   /**< @brief Lower soft limit position for Elevation (value -1 indicates unused or physical limit). */
///@}