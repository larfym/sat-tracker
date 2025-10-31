#include <Sgp4.h>
#include <Preferences.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

extern Sgp4 satellite;
extern TinyGPSPlus gps;
extern Preferences satellite_preferences;

extern String estado;
extern double offset_az, offset_el;

extern bool gps_initialized, tle_initialized, time_initialized;

extern unsigned long unixtime;