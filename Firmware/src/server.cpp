#include "server.h"

static const char *TAG = "SERVER";

ServerHandler::ServerHandler(int port) : server(port), events("/telemetry")
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

    // Telemetry (SSE)
    server.addHandler(&events);
    events.onConnect([](AsyncEventSourceClient *client) {
        ESP_LOGI(TAG, "SSE client connected");
        client->send("Connected", NULL, millis(), 1000);
    });
    events.onDisconnect([](AsyncEventSource *source, AsyncEventSourceClient *client) {
        ESP_LOGI(TAG, "SSE client disconnected");
    });

    // Tracking (POST /toggle_tracking)
    server.on("/toggle_tracking", HTTP_POST, std::bind(&ServerHandler::toggleTracking_handle, this, std::placeholders::_1));

    // Configuration (POST /saveTle)
    server.on("/saveTle", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, std::bind(&ServerHandler::saveTLE_handle, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
    
    // Configuration (POST /saveOffsets)
    server.on("/saveOffsets", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, std::bind(&ServerHandler::saveOffsets_handle, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

    // GeoTime (POST /geo_time)
    server.on("/geo_time", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, std::bind(&ServerHandler::GeoTime_handle, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

    // Manual Control (POST /manual)
    server.on("/manual", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, std::bind(&ServerHandler::manualTrack_handle, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

    // Reset MCU (POST /reset)
    server.on("/reset", HTTP_POST, std::bind(&ServerHandler::reset_handle, this, std::placeholders::_1));

    // Not Found
    server.onNotFound(NotFount_handle);

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

void ServerHandler::saveTLE_handle(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
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
        flags.tle_updated = true;
        request->send(200, "text/plain", "OK");
    }
}

void ServerHandler::saveOffsets_handle(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
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

        saveOffsets(offset_az.toFloat(), offset_el.toFloat());
        flags.offsets_updated = true;
        request->send(200, "text/plain", "OK");
    }
}

void ServerHandler::toggleTracking_handle(AsyncWebServerRequest *request)
{
    JsonDocument res;
    if (status.tracking == true)
    {
        status.tracking = false;
        res["tracking"] = false;
        res["reason"] = "none";
        
        if (status.manual_tracking == true)
        {
            status.manual_tracking = false;
        }
        ESP_LOGI(TAG, "Tracking Stopped");
    }
    else
    {
        if (status.gps == true && status.tle == true)
        {
            status.tracking = true;
            res["tracking"] = status.tracking;
            res["reason"] = "none";
            ESP_LOGI(TAG, "Tracking Started");
        }
        else if (status.gps == false)
        {
            res["tracking"] = false;
            res["reason"] = "No_GPS";
            ESP_LOGI(TAG, "Cannot start tracking: No GPS Fix");
        }
        else if (status.tle == false)
        {
            res["tracking"] = false;
            res["reason"] = "No_TLE";
            ESP_LOGI(TAG, "Cannot start tracking: TLE Not Initialized");
        }
    }

    ESP_LOGD(TAG, "Status - Tracking: %d, Manual Track: %d, GPS Fix: %d, TLE Inited: %d", status.tracking, status.manual_tracking, status.gps, status.tle);
    String _res;
    serializeJson(res, _res);
    request->send(200, "application/json", _res);
}

void ServerHandler::manualTrack_handle(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
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

        status.manual_tracking = true;
        status.tracking = true;
        float az = doc["manual_az"];
        float el = doc["manual_el"];
        manual_target.azimuth = az;
        manual_target.elevation = el;

        ESP_LOGI(TAG, "Manual Track Set to Az: %f, El: %f", az, el);
        ESP_LOGD(TAG, "Status - Tracking: %d, Manual Track: %d, GPS Fix: %d, TLE Inited: %d", status.tracking, status.manual_tracking, status.gps, status.tle);
        request->send(200, "text/plain", "OK");
    }
    else
    {
        request->send(400, "text/plain", "Invalid Request");
    }
}

void ServerHandler::GeoTime_handle(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
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

        setTime(unix_time);
        satellite.site(latitude, longitude, altitude);
        status.gps = true;

        ESP_LOGD(TAG, "GeoTime Updated - Time: %lu, Lat: %f, Lon: %f, Alt: %f", unix_time, latitude, longitude, altitude);
        request->send(200, "text/plain", "OK");
    }
}

void ServerHandler::NotFount_handle(AsyncWebServerRequest *request)
{
    request->send(404, "text/plain", "Not Found");
}

void ServerHandler::reset_handle(AsyncWebServerRequest *request)
{
    flags.reset_pending = true;
    request->send(200, "text/plain", "OK");
}

void ServerHandler::sendTelemetry()
{
    JsonDocument json;

    //Satellite Data
    JsonObject sat = json["s"].to<JsonObject>(); 
    sat["n"] = satellite.satName;
    sat["la"] = serialized(String(satellite.satLat, 6));
    sat["lo"] = serialized(String(satellite.satLon, 6));
    sat["al"] = serialized(String(satellite.satAlt, 2));
    sat["a"] = serialized(String(satellite.satAz, 2));
    sat["e"] = serialized(String(satellite.satEl, 2));
    sat["p"] = next_pass_unix;
    
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
    
    //Status Data
    JsonObject stat = json["st"].to<JsonObject>(); 
    stat["tle"] = status.tle;
    stat["gps"] = status.gps;
    stat["man"] = status.manual_tracking;
    stat["tr"] = status.tracking;
    stat["err"] = status.error;
    stat["a"] = serialized(String(set_angle.azimuth, 2));
    stat["e"] = serialized(String(set_angle.elevation, 2));

    //Antenna Data
    JsonObject ant = json["a"].to<JsonObject>(); 
    ant["o_a"] = serialized(String(offsets_ant.azimuth, 2));
    ant["o_e"] = serialized(String(offsets_ant.elevation, 2));

    //Platform Data
    unsigned long unixtime = time(NULL);
    JsonObject platform = json["p"].to<JsonObject>(); 
    platform["la"] = serialized(String(satellite.siteLat, 6));
    platform["lo"] = serialized(String(satellite.siteLon, 6));
    platform["al"] = serialized(String(satellite.siteAlt, 2));
    platform["az"] = serialized(String(current.azimuth, 2));
    platform["el"] = serialized(String(current.elevation, 2));
    if(status.gps){
        platform["t"] = unixtime;
    }else{
        platform["t"] = 0.0f;
    }
    
    String output;
    serializeJson(json, output);
    events.send(output.c_str(), "telemetry", millis());
}