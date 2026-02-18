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

    // Configuración (POST /saveTle)
    server.on("/saveTle", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, std::bind(&ServerHandler::handleSaveTle, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
    
    // Configuración (POST /saveOffsets)
    server.on("/saveOffsets", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, std::bind(&ServerHandler::handleSaveOffsets, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

    // GeoTime (POST /geo_time)
    server.on("/geo_time", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, std::bind(&ServerHandler::handleGeoTime, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

    // Control Manual (POST /manual)
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
    JsonDocument json;
    char temp_float_buffer[FLOAT_BUFFER_SIZE];

    JsonObject sat = json["s"].to<JsonObject>(); //Satellite Data
    sat["n"] = satellite.satName;
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.6f", satellite.satLat);
    sat["la"] = temp_float_buffer;
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.6f", satellite.satLon);
    sat["lo"] = temp_float_buffer;
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.2f", satellite.satAlt);
    sat["al"] = temp_float_buffer;
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.3f", satellite.satAz);
    sat["a"] = temp_float_buffer;
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.3f", satellite.satEl);
    sat["e"] = temp_float_buffer;
    
    bool notdark;
    double deltaphi;
    int16_t vis = satellite.visible(notdark, deltaphi);

    if (satellite.satEl <= 0) {
        sat["v"] = "No visible";
    }
    else if (notdark) {
        sat["v"] = "Iluminado";
    }
    else if (vis == 0) {
        sat["v"] = "Eclipse";
    }
    else if (vis < 1000) {
        sat["v"] = "Penumbra";
    }
    else {
        sat["v"] = "Visible";
    }
    
    JsonObject stat = json["st"].to<JsonObject>(); //Status Data
    stat["tle"] = status.tle_inited;
    stat["gps"] = status.gps_fix;
    stat["man"] = status.manual_track;
    stat["tr"] = status.tracking;
    stat["err"] = status.error;
    stat["a"] = set_angle.azimuth;
    stat["e"] = set_angle.elevation;
    
    JsonObject ant = json["a"].to<JsonObject>(); //Antenna Data
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.2f", offsets_ant.azimuth);
    ant["o_a"] = temp_float_buffer;
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.2f", offsets_ant.elevation);
    ant["o_e"] = temp_float_buffer;

    unsigned long unixtime = time(NULL);
    JsonObject platform = json["p"].to<JsonObject>(); //Platform Data
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.2f", current.azimuth);
    platform["az"] = temp_float_buffer;
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.2f", current.elevation);
    platform["el"] = temp_float_buffer;
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.6f", satellite.siteLat);
    platform["la"] = temp_float_buffer;
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.6f", satellite.siteLon);
    platform["lo"] = temp_float_buffer;
    snprintf(temp_float_buffer, FLOAT_BUFFER_SIZE, "%.2f", satellite.siteAlt);
    platform["al"] = temp_float_buffer;
    if(status.gps_fix){
        platform["t"] = unixtime;
    }else{
        platform["t"] = 0;
    }
    
    char buffer[JSON_BUFFER_SIZE];
    serializeJson(json, buffer);
    request->send(200, "application/json", buffer);
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
    JsonDocument res;
    if (status.tracking == true)
    {
        status.tracking = false;
        res["tracking"] = false;
        res["reason"] = "none";
        
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
            res["tracking"] = status.tracking;
            res["reason"] = "none";
            ESP_LOGI(TAG, "Tracking Started");
        }
        else if (status.gps_fix == false)
        {
            res["tracking"] = false;
            res["reason"] = "No_GPS";
            ESP_LOGI(TAG, "Cannot start tracking: No GPS Fix");
        }
        else if (status.tle_inited == false)
        {
            res["tracking"] = false;
            res["reason"] = "No_TLE";
            ESP_LOGI(TAG, "Cannot start tracking: TLE Not Initialized");
        }
    }

    ESP_LOGD(TAG, "Status - Tracking: %d, Manual Track: %d, GPS Fix: %d, TLE Inited: %d", status.tracking, status.manual_track, status.gps_fix, status.tle_inited);
    String _res;
    serializeJson(res, _res);
    request->send(200, "application/json", _res);
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
