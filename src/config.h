#pragma once
#include <Arduino.h>

#define SERIAL_BAUDRATE 115200
#define DEFAULT_WIFI_SSID "ESP32_AP"
#define DEFAULT_WIFI_PASSWORD ""
#define SERVER_PORT 80
#define MDNS_SERVICE_NAME "Tracker"
#define WIFI_CONNECT_TIMEOUT 10000

// Default values for satellite position and time
#define DEFAULT_LATITUD 37.7749
#define DEFAULT_LONGITUD -122.4194
#define DEFAULT_ALTITUD 0
#define DEFAULT_UNIXTIME 1750015604

//Timer Values
#define SECOND_TIMER_PREESCALER 80
#define SECOND_TIMER_ALARM_VALUE 1000000

//GPS
#define GPS_UART 1
#define GPS_BAUDRATE 9600
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_INTERVAL_SEC 120