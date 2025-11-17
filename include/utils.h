#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <stdint.h>
#include <Preferences.h>
#include <Sgp4.h>
#include <TinyGPSPlus.h>
#include <sys/time.h>
#include <math.h>
#include <stdint.h>
#include "driver/timer.h"
#include "configurations.h"

#include "esp_log.h"
#include "esp_err.h"

// --------------------------------------------------------------------------------------
//                                 CONSTANTS
// --------------------------------------------------------------------------------------

/** @brief Lookup Table (LUT) size for elevation calculation. */
#define LUT_SIZE 45

/** @brief PI constant in float precision. */
#define PI_FLOAT 3.1415926535f

/** @brief Angular step in radians for the LUT (90° / LUT_SIZE). */
#define ANGLE_STEP ((PI_FLOAT / 2.0f) / (float)LUT_SIZE)

/** @brief Precomputed factor to convert degrees to LUT index. */
#define INV_DEG_TO_LUT_POS ((float)LUT_SIZE / 90.0f)

// Antenna mechanical constants (mm)
#define A_ANT 55.73f
#define BP_ANT 105.45f
#define C_ANT 276.90f
#define D_ANT 138.91f
#define E_ANT 80.0f
#define F_ANT 262.27f

// --------------------------------------------------------------------------------------
//                                 GLOBAL REFERENCES
// --------------------------------------------------------------------------------------

/**
 * @brief External reference to the global Preferences storage handler.
 */
extern Preferences config;

// --------------------------------------------------------------------------------------
//                                  DATA STRUCTURES
// --------------------------------------------------------------------------------------

/**
 * @brief Status flags for the satellite tracking system.
 *
 * Uses bit-fields to pack multiple status indicators and a 3-bit error code
 * into a single byte.
 */
typedef struct __attribute__((packed))
{
    uint8_t tle_inited : 1;     /**< True if TLE data has been successfully loaded. */
    uint8_t gps_fix : 1;        /**< True if GPS has a valid fix. */
    uint8_t tle_changed : 1;    /**< True if TLE data has been updated. */
    uint8_t manual_track : 1;   /**< True if antenna is in manual control mode. */
    uint8_t tracking : 1;       /**< True if automatic tracking is active. */
    uint8_t error : 3;          /**< 3-bit internal error code. */
} trackerStatus_t;

/**
 * @brief Represents the angular position of the antenna.
 */
typedef struct __attribute__((packed))
{
    float azimuth;      /**< Azimuth angle in degrees (0–360). */
    float elevation;    /**< Elevation angle in degrees (0–90). */
} antennaPosition_t;

/**
 * @brief Direction type for counting or motor rotation.
 */
typedef enum
{
    FORWARD,    /**< Positive direction. */
    BACKWARD    /**< Negative direction. */
} direction;

// --------------------------------------------------------------------------------------
//                                   FUNCTION HEADERS
// --------------------------------------------------------------------------------------

/**
 * @brief Loads TLE data from NVS and configures the given Sgp4 object.
 *
 * @param satellite Pointer to the Sgp4 object to configure.
 * @return true if TLE data was successfully loaded and applied, false otherwise.
 */
bool configSatellite(Sgp4 *satellite);

/**
 * @brief Reads saved azimuth and elevation offsets from NVS.
 *
 * @return A structure containing the stored azimuth and elevation offsets.
 */
antennaPosition_t getSavedOffsets(void);

/**
 * @brief Extracts the current Unix timestamp from a TinyGPSPlus instance.
 *
 * @param gps Pointer to a valid TinyGPSPlus object.
 * @return Unix time in seconds since January 1, 1970.
 */
unsigned long getUnixTimeFromGPS(TinyGPSPlus *gps);

/**
 * @brief Configures an ESP32 hardware timer to generate periodic interrupts.
 *
 * @param timerISR Pointer to the function executed on each timer interrupt.
 * @param sample_time_us Timer period in microseconds (default: DEFAULT_SAMPLE_TIME_US).
 */
void configControlTimer(void (*timerISR)(void *), uint64_t sample_time_us = DEFAULT_SAMPLE_TIME_US);

/**
 * @brief Saves TLE data to NVS and updates the Sgp4 object.
 *
 * @param name Satellite name.
 * @param line1 First TLE line.
 * @param line2 Second TLE line.
 * @param offset_az Azimuth offset to be saved.
 * @param offset_el Elevation offset to be saved.
 */
void saveTLEdata(String name, String line1, String line2, double offset_az, double offset_el);

/**
 * @brief Initializes the elevation lookup table (LUT).
 */
void init_elevation_lut();

/**
 * @brief Retrieves a distance value from the LUT based on the input angle.
 *
 * @param angle_deg Angle in degrees.
 * @return Interpolated distance from the LUT.
 */
float get_distance_from_lut(float angle_deg);

#endif // UTILS_H
