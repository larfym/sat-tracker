#include "server.h"

static const char *TAG = "SERVER";

ServerHandler::ServerHandler(int port) : server(port)
{
}

esp_err_t ServerHandler::start()
{
    if (!SPIFFS.begin(true))
    {
        ESP_LOGE(TAG, "An error has occurred while mounting SPIFFS");
        delay(2000);
        ESP_LOGI(TAG, "Reseting...");
        ESP.restart();
    }

    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP(default_wifi_ssid.c_str(), default_wifi_password.c_str());

    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    // Telemetría (GET /data)
    server.on("/data", HTTP_GET,
              std::bind(&ServerHandler::handleData, this, std::placeholders::_1));

    // Trackeo (POST /toggle_tracking)
    server.on("/toggle_tracking", HTTP_POST,
              std::bind(&ServerHandler::handleToggleTracking, this, std::placeholders::_1));

    // Configuración (POST /saveTle) - Requiere manejo de cuerpo (body)
    server.on("/saveTle", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, std::bind(&ServerHandler::handleSaveTle, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
    
        // Configuración (POST /saveOffsets) - Requiere manejo de cuerpo (body)
    server.on("/saveOffsets", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, std::bind(&ServerHandler::handleSaveOffsets, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

    server.on("/geo_time", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, std::bind(&ServerHandler::handleGeoTime, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

    // Control Manual (POST /manual) - Requiere manejo de cuerpo (body)
    server.on("/manual", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, std::bind(&ServerHandler::handleManualTrack, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

    server.onNotFound(handleNotFound);

    if (MDNS.begin(MDNS_SERVICE_NAME))
    {
        MDNS.addService("http", "tcp", DEFAULT_SERVER_PORT);
        ESP_LOGI(TAG, "mDNS Started on: http://%s.local", MDNS_SERVICE_NAME);
        server.begin();
    }
    else
    {
        ESP_LOGE(TAG, "An error has occurred while starting mDNS");
        return ESP_FAIL;
    }

    server.begin();
    ESP_LOGI(TAG, "Initialized");
    return ESP_OK;
}

void ServerHandler::handleData(AsyncWebServerRequest *request)
{
    StaticJsonDocument<JSON_BUFFER_SIZE> doc;
    char jsonbuffer[JSON_BUFFER_SIZE];
    char temp_float_buffer[FLOAT_BUFFER_SIZE];

    if (status.tle_inited)
    {
        doc["name"] = satellite.satName;
        snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.6f", satellite.satLon);
        doc["s_lon"] = temp_float_buffer;
        snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.6f", satellite.satLat);
        doc["s_lat"] = temp_float_buffer;
        snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.4f", satellite.satAlt);
        doc["s_alt"] = temp_float_buffer;
    }
    else
    {
        doc["stat"] = "No TLE";
    }
    if (status.gps_fix)
    {
        unsigned long unixtime = time(NULL);
        snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.6f", satellite.siteLon);
        doc["t_lon"] = temp_float_buffer;
        snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.6f", satellite.siteLat);
        doc["t_lat"] = temp_float_buffer;
        snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.4f", satellite.siteAlt);
        doc["t_alt"] = temp_float_buffer;
        doc["t_tim"] = unixtime;
    }
    else
    {
        doc["stat"] = "No GPS";
    }
    if (status.gps_fix && status.tle_inited)
    {
        doc["s_az"] = target.azimuth;
        doc["s_el"] = target.elevation;
    }

    doc["a_az"] = current.azimuth;
    doc["a_el"] = current.elevation;
    doc["az_off"] = offsets_ant.azimuth;
    doc["el_off"] = offsets_ant.elevation;

    if (status.tracking)
    {
        doc["stat"] = "TRACKING";
    }
    if (status.manual_track)
    {
        doc["stat"] = "MANUAL";
    }

    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
}

void ServerHandler::handleSaveTle(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
    if (index == total - len && request->contentType() == "application/json")
    {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, data, total);
        if (err)
        {
            ESP_LOGE(TAG, "Failed to parse JSON: %s", err.c_str());
            request->send(400, "text/plain", "Invalid JSON");
            return;
        }

        String name = doc["name"];
        String line1 = doc["line1"];
        String line2 = doc["line2"];

        saveTLE(name, line1, line2);
        status.tle_changed = true;
        request->send(200, "text/plain", "OK");
    }
}

void ServerHandler::handleSaveOffsets(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
    if (index == total - len && request->contentType() == "application/json")
    {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, data, total);
        if (err)
        {
            ESP_LOGE(TAG, "Failed to parse JSON: %s", err.c_str());
            request->send(400, "text/plain", "Invalid JSON");
            return;
        }

        String offset_az = doc["offset_az"];
        String offset_el = doc["offset_el"];

        saveOffsets(offset_az.toDouble(), offset_el.toDouble());
        status.offsets_changed = true;
        request->send(200, "text/plain", "OK");
    }
}

void ServerHandler::handleToggleTracking(AsyncWebServerRequest *request)
{
    JsonDocument doc;
    if (status.tracking == true)
    {
        doc["tracking_active"] = false;
        doc["reason"] = "none";
        status.tracking = false;
        if (status.manual_track == true)
        {
            status.manual_track = false;
        }
        ESP_LOGI(TAG, "Tracking Stopped");
    }
    else
    {
        if (status.gps_fix == true && status.tle_inited == true)
        {
            status.tracking = true;
            doc["tracking_active"] = status.tracking;
            doc["reason"] = "none";
            ESP_LOGI(TAG, "Tracking Started");
        }
        else if (status.gps_fix == false)
        {
            doc["tracking_active"] = false;
            doc["reason"] = "No_GPS";
            ESP_LOGI(TAG, "Cannot start tracking: No GPS Fix");
        }
        else if (status.tle_inited == false)
        {
            doc["tracking_active"] = false;
            doc["reason"] = "No_TLE";
            ESP_LOGI(TAG, "Cannot start tracking: TLE Not Initialized");
        }
    }

    ESP_LOGD(TAG, "Status - Tracking: %d, Manual Track: %d, GPS Fix: %d, TLE Inited: %d", status.tracking, status.manual_track, status.gps_fix, status.tle_inited);
    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
}

void ServerHandler::handleManualTrack(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
    if (index == total - len && request->contentType() == "application/json")
    {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, (const char *)data);
        if (err)
        {
            request->send(400, "text/plain", "Invalid JSON");
            return;
        }

        status.manual_track = true;
        status.tracking = true;
        float az = doc["manual_az"];
        float el = doc["manual_el"];
        manual_target.azimuth = az;
        manual_target.elevation = el;

        ESP_LOGI(TAG, "Manual Track Set to Az: %f, El: %f", az, el);
        request->send(200, "text/plain", "OK");
    }
    else
    {
        request->send(400, "text/plain", "Invalid Request");
    }
}

void ServerHandler::handleGeoTime(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
    if (index == total - len && request->contentType() == "application/json")
    {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, (const char *)data);
        if (err)
        {
            request->send(400, "text/plain", "Invalid JSON");
            return;
        }

        unsigned long unix_time = doc["unix"];
        double latitude = doc["lat"];
        double longitude = doc["lon"];
        double altitude = doc["alt"];

        struct timeval tv;
        tv.tv_sec = unix_time;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);

        satellite.site(latitude, longitude, altitude);
        status.gps_fix = true;

        ESP_LOGI(TAG, "GeoTime Updated - Time: %lu, Lat: %f, Lon: %f, Alt: %f", unix_time, latitude, longitude, altitude);
        request->send(200, "text/plain", "OK");
    }
}

void ServerHandler::handleNotFound(AsyncWebServerRequest *request)
{
    request->send(404, "text/plain", "Not Found");
}
