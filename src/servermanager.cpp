#include "servermanager.h"
#include "config.h"
#include "main.h"

AsyncWebServer server(SERVER_PORT);

const String default_wifi_ssid = DEFAULT_WIFI_SSID;
const String default_wifi_password = DEFAULT_WIFI_PASSWORD;

volatile bool apStarted = false;

void serverBegin()
{
    __startAPMode();
    __startTrackerServer();
}

void serverHandle()
{
    if (MDNS.begin(MDNS_SERVICE_NAME))
    {
        Serial.printf("mDNS iniciado: http://%s.local\n", MDNS_SERVICE_NAME);
        MDNS.addService("http", "tcp", SERVER_PORT);
    }
    else
    {
        Serial.println("Error al iniciar el servidor mDNS");
    }

    server.begin();
}

void __startAPMode()
{
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP(default_wifi_ssid.c_str(), default_wifi_password.c_str());
}

void __startTrackerServer()
{
    Serial.println("Servidor de aplicación iniciado");

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", "text/html"); });

    server.on("/fondo.jpg", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/fondo.jpg", "image/jpeg"); });

    server.on("/data", HTTP_GET, __handleTrackerData);

    server.on("/track", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, __handleTrackerConfigSave);
}

void __handleTrackerConfigSave(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
    String body = "";
    for (size_t i = 0; i < len; i++)
    {
        body += (char)data[i];
    }

    auto extractValue = [&](const String &key) -> String
    {
        int keyIndex = body.indexOf("\"" + key + "\"");
        if (keyIndex == -1)
            return "";

        int colonIndex = body.indexOf(':', keyIndex);
        if (colonIndex == -1)
            return "";

        int valueStart = colonIndex + 1;

        // Saltar espacios
        while (valueStart < body.length() && isspace(body[valueStart]))
        {
            valueStart++;
        }

        int valueEnd = valueStart;

        // Buscar fin del valor (coma, llave o comilla de cierre)
        bool isQuoted = body[valueStart] == '"';
        if (isQuoted)
        {
            valueStart++; // saltar comilla inicial
            valueEnd = body.indexOf('"', valueStart);
        }
        else
        {
            while (valueEnd < body.length() && body[valueEnd] != ',' && body[valueEnd] != '}')
            {
                valueEnd++;
            }
        }

        if (valueEnd == -1 || valueEnd <= valueStart)
            return "";

        String valueStr = body.substring(valueStart, valueEnd);
        valueStr.trim();
        return valueStr;
    };

    String tle = extractValue("tle");
    tle.replace("\\n", "\n");
    int nl1 = tle.indexOf('\n');
    int nl2 = tle.indexOf('\n', nl1 + 1);

    if (nl1 == -1 || nl2 == -1)
    {
        request->send(500, "text/plain", "Error: TLE inválido");
        return;
    }

    satellite_preferences.putString("name", tle.substring(0, nl1));
    satellite_preferences.putString("line1", tle.substring(nl1 + 1, nl2));
    satellite_preferences.putString("line2", tle.substring(nl2 + 1));
    satellite_preferences.putDouble("offset_az", extractValue("offset_az").toFloat());
    satellite_preferences.putDouble("offset_el", extractValue("offset_el").toFloat());

    tle_initialized = false;

    request->send(200, "text/plain", "OK");
}

void __handleTrackerData(AsyncWebServerRequest *request)
{
    String name, latitud_sat, longitud_sat, altitud_sat, latitud_ant, longitud_ant, altitud_ant, time, az, el;
    if(time_initialized)
    {
        time = __formatUnixTime(unixtime);
    }else
    {
        time = "--";
    }
    if (gps_initialized && tle_initialized)
    {
        name = String(satellite.satName);
        latitud_sat = String(satellite.satLat, 6);
        longitud_sat = String(satellite.satLon, 6);
        altitud_sat = String(satellite.satAlt, 4) + " [km]";
        latitud_ant = String(satellite.siteLat, 6);
        longitud_ant = String(satellite.siteLon, 6);
        altitud_ant = String(satellite.siteAlt, 4) + " [km]";
        az = String(satellite.satAz, 3);
        el = String(satellite.satEl, 3);
    }
    else if (tle_initialized && !gps_initialized)
    {
        name = String(satellite.satName);
        latitud_sat = "--";
        longitud_ant = "--";
        altitud_sat = "--";
        latitud_ant = "--";
        longitud_ant = "--";
        altitud_ant = "--";
        az = "--";
        el = "--";
    }
    else if (!tle_initialized && gps_initialized)
    {
        name = "--";
        latitud_sat = "--";
        longitud_ant = "--";
        altitud_sat = "--";
        latitud_ant = String(satellite.siteLat, 6);
        longitud_ant = String(satellite.siteLon, 6);
        altitud_ant = String(satellite.siteAlt, 4) + " [km]";
        az = "--";
        el = "--";
    }
    else
    {
        name = "--";
        latitud_sat = "--";
        longitud_ant = "--";
        altitud_sat = "--";
        latitud_ant = "--";
        longitud_ant = "--";
        altitud_ant = "--";
        az = "--";
        el = "--";
    }

    String json = "{";
    json += "\"nombre\":\"" + name + "\",";
    json += "\"latitud_sat\":\"" + latitud_sat + "\",";
    json += "\"longitud_sat\":\"" + longitud_sat + "\",";
    json += "\"altitud_sat\":\"" + altitud_sat + "\",";
    json += "\"latitud_ant\":\"" + latitud_ant + "\",";
    json += "\"longitud_ant\":\"" + longitud_ant + "\",";
    json += "\"altitud_ant\":\"" + altitud_ant + "\",";
    json += "\"tiempo\":\"" + time + "\",";
    json += "\"azimuth\":\"" + az + "\",";
    json += "\"elevacion\":\"" + el + "\",";
    json += "\"estado\":\"" + String(estado) + "\"";
    json += "}";

    request->send(200, "application/json", json);
}

String __formatUnixTime(unsigned long unix)
{
    time_t t = (time_t)unix;
    struct tm *tmStruct = localtime(&t);
    char buff[32];
    strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", tmStruct);
    return String(buff);
}


