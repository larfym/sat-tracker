#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <stdint.h>
#include <Preferences.h>
#include <Sgp4.h>
#include <TinyGPSPlus.h>
#include <sys/time.h>
#include "driver/timer.h"
#include "configurations.h"

#include "esp_log.h"
#include "esp_err.h"

/** @brief External reference to the global Preferences object for NVS (Non-Volatile Storage) access. */
extern Preferences config;

/**
 * @brief Structure to hold the operational status of the satellite tracker.
 *
 * Uses bit-fields to efficiently store multiple boolean flags and a small error code
 * within a single byte (packed attribute ensures minimal memory usage).
 */
typedef struct __attribute__((packed))
{
    uint8_t tle_inited : 1;     /**< Bit 0: True if TLE data has been successfully loaded. */
    uint8_t gps_fix : 1;        /**< Bit 1: True if the GPS module has a valid position fix. */
    uint8_t tle_changed : 1;    /**< Bit 2: True if the TLE data has been updated since the last check. */
    uint8_t manual_track : 1;   /**< Bit 3: True if the antenna is being controlled manually. */
    uint8_t tracking : 1;       /**< Bit 4: True if automatic tracking is currently active. */
    uint8_t error : 3;          /**< Bits 5-7: Status code for any internal error. */
} trackerStatus_t;

/**
 * @brief Structure to represent the angular position of the antenna.
 *
 * Stores the Azimuth and Elevation angles as floating-point numbers.
 */
typedef struct __attribute__((packed))
{
    float azimuth;      /**< Azimuth angle in degrees (0 to 360). */
    float elevation;    /**< Elevation angle in degrees (0 to 90). */
} antennaPosition_t;

/**
 * @brief Enumeration for defining direction (e.g., motor rotation, counting direction).
 */
typedef enum
{
    FORWARD,    /**< Moving or counting in the forward/positive direction. */
    BACKWARD    /**< Moving or counting in the backward/negative direction. */
} direction;

/**
 * @brief Configures the Sgp4 object with TLE data retrieved from NVS (Non-Volatile Storage).
 * @param satellite Pointer to the Sgp4 object to be configured.
 * @return true if TLE data was successfully loaded and configured, false otherwise.
 */
bool configSatellite(Sgp4 *satellite);

/**
 * @brief Retrieves the saved azimuth and elevation offset values from NVS.
 * @return An antennaPosition_t structure containing the saved offset angles.
 */
antennaPosition_t getSavedOffsets(void);

/**
 * @brief Extracts the current Unix timestamp from the TinyGPSPlus object.
 * @param gps Pointer to the initialized TinyGPSPlus object.
 * @return The current time in seconds since the Unix epoch (January 1, 1970).
 */
unsigned long getUnixTimeFromGPS(TinyGPSPlus *gps);

/**
 * @brief Configures the ESP32's hardware timer to fire an interrupt every second.
 * @param timerISR The callback function to execute upon the one-second timer interrupt.
 * @param sample_time_us The timer interval in microseconds (default is 100000 for one 100 mS).
 */
void configControlTimer(void (*timerISR)(void *), uint64_t sample_time_us = DEFAULT_SAMPLE_TIME_US);

/**
 * @brief Utility function to save TLE data (Two-Line Elements) to NVS and update Sgp4.
 * @param name The name of the satellite.
 * @param line1 The first TLE line.
 * @param line2 The second TLE line.
 * @param offset_az Azimuth offset to save.
 * @param offset_el Elevation offset to save.
 */
void saveTLEdata(String name, String line1, String line2, double offset_az, double offset_el);

#endif // UTILS_H