#ifndef SERVER_HANDLER_H
#define SERVER_HANDLER_H

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Sgp4.h>
#include <SPIFFS.h>
#include "utils.h"
#include "esp_log.h"
#include "esp_err.h"

/** @brief Default port for the web server (HTTP). */
#define DEFAULT_SERVER_PORT 80
/** @brief mDNS service name for device discovery (e.g., "Tracker.local"). */
#define MDNS_SERVICE_NAME "Tracker"
/** @brief Default SSID for the ESP32's Access Point (AP) mode. */
#define DEFAULT_WIFI_SSID "SatTracker"
/** @brief Default password for the ESP32's Access Point (AP) mode. */
#define DEFAULT_WIFI_PASSWORD "12345678"

/** @brief Buffer size for formatting floating-point numbers as strings. */
#define FLOAT_BUFFER_SIZE 16

// ---------------------------
// 1. GLOBAL STATE VARIABLES
// ---------------------------
/** @brief Global SGP4 object for satellite orbit prediction. */
extern Sgp4 satellite;
/** @brief Global object for managing persistent configuration data (NVS). */
extern Preferences config;
/** @brief Global structure holding the current operational status of the tracker. */
extern trackerStatus_t status;
/** @brief Global structure for the target (commanded) antenna position. */
extern antennaPosition_t tar_angle;
/** @brief Global structure for the current (measured) antenna position. */
extern antennaPosition_t curr_angle;
/** @brief Global variable for the current Unixtime (seconds since 1970). */
extern unsigned long unixtime;

/** @brief Constant string for the default WiFi SSID. */
const String default_wifi_ssid = DEFAULT_WIFI_SSID;
/** @brief Constant string for the default WiFi password. */
const String default_wifi_password = DEFAULT_WIFI_PASSWORD;
/** @brief Size of the ArduinoJson buffer used for serializing/deserializing data. */
const size_t JSON_BUFFER_SIZE = 384;

/**
 * @class ServerHandler
 * @brief Manages the ESP32's asynchronous web server, handles API requests,
 * and serves configuration/status data.
 *
 * This class sets up the HTTP server, defines endpoints for data retrieval,
 * configuration saving, manual control, and tracking status toggling.
 */
class ServerHandler
{
public:
    /**
     * @brief Constructor for the ServerHandler class.
     * @param port The HTTP port to run the server on (defaults to 80).
     */
    ServerHandler(int port = DEFAULT_SERVER_PORT);

    /**
     * @brief Starts the web server and configures the mDNS service.
     * @return ESP_OK on success, or an error code on failure.
     */
    esp_err_t start();

private:
    AsyncWebServer server; /**< The asynchronous web server instance. */

    /**
     * @brief Handler for retrieving status and configuration data (GET request).
     * @param request The asynchronous request object.
     */
    void handleData(AsyncWebServerRequest *request);

    /**
     * @brief Handler for saving satellite configuration data (tle and offsets) (POST request).
     * @param request The asynchronous request object.
     * @param data The received POST data payload.
     * @param len Length of the data payload.
     * @param index Current index of the received chunk (for large uploads).
     * @param total Total length of the expected payload.
     */
    void handleSave(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);

    /**
     * @brief Handler for setting manual target angles for the platform (POST request).
     * @param request The asynchronous request object.
     * @param data The received POST data payload.
     * @param len Length of the data payload.
     * @param index Current index of the received chunk.
     * @param total Total length of the expected payload.
     */
    void handleManualTrack(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);

    /**
     * @brief Handler for toggling the automatic tracking state (GET request).
     * @param request The asynchronous request object.
     */
    void handleToggleTracking(AsyncWebServerRequest *request);

    /**
     * @brief Static handler for managing "404 Not Found" errors.
     * @param request The asynchronous request object.
     */
    static void handleNotFound(AsyncWebServerRequest *request);
};

/**
 * @brief Helper function to format a Unix timestamp into a readable string (e.g., for logging/debugging).
 * @param unix The Unix timestamp (in seconds).
 * @return A String containing the formatted time.
 */
String __formatUnixTime(unsigned long unix);//TEST

#endif // SERVER_HANDLER_H