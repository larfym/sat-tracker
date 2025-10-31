#pragma once

#include "config.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <HTTPClient.h>

void serverBegin();
void serverHandle();

void __startAPMode();
void __startTrackerServer();
void __handleTrackerConfigSave(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void __handleTrackerData(AsyncWebServerRequest *request);
String __formatUnixTime(unsigned long unixTime);