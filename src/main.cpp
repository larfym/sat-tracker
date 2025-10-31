#include "config.h"
#include "servermanager.h"
#include "main.h"

Sgp4 satellite;
TinyGPSPlus gps;
HardwareSerial gpsSerial(GPS_UART);

Preferences satellite_preferences;
String estado = "";
unsigned long unixtime = DEFAULT_UNIXTIME;
volatile uint32_t gps_interval_sec = 0;
double offset_az, offset_el;

hw_timer_t *timer = NULL;

bool gps_initialized = false, tle_initialized = false, time_initialized = false;
bool gps_getData = false;

void initSecondTimer();
void IRAM_ATTR secondTimer();
unsigned long getUnixTimeFromGPS();
void configSatellite();

void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  if (!SPIFFS.begin(true))
  {
    Serial.println("Error al montar SPIFFS");
    ESP.restart();
  }

  configSatellite();
  serverBegin();
  serverHandle();
  initSecondTimer();
}

void loop()
{
  if(gps_getData)
  {
    while(gpsSerial.available() > 0)
    {
      gps.encode(gpsSerial.read());
    }
  }

  if(gps.time.isUpdated() && gps.date.isUpdated())
  {
    unixtime = getUnixTimeFromGPS();
    time_initialized = true;
  }

  if(gps.location.isUpdated())
  {
    if(gps.location.isValid())
    {
      gps_getData = false;
      gps_initialized = true;
      satellite.site(gps.location.lat(), gps.location.lng(), gps.altitude.meters());

      Serial.print("Latitud: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitud: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("Altitud: ");
      Serial.print(gps.altitude.meters());
      Serial.println(" m");
    }
  }

  if(tle_initialized)
  {
    if(gps_initialized)
    {
      estado = "Trackeando";
      satellite.findsat(unixtime);
    }else
    {
      estado = "Obteniendo datos del GPS";
    }
  }else
  {
    configSatellite();
    estado = "sin datos TLE";
  }

}

void initSecondTimer()
{
  timer = timerBegin(0, SECOND_TIMER_PREESCALER, true);
  timerAttachInterrupt(timer, &secondTimer, true);
  timerAlarmWrite(timer, SECOND_TIMER_ALARM_VALUE, true);
  timerAlarmEnable(timer);
}

unsigned long getUnixTimeFromGPS()
{
  if(gps.date.isValid() && gps.time.isValid())
  {
    struct tm t;
    t.tm_year = gps.date.year() - 1900;
    t.tm_mon = gps.date.month() - 1;
    t.tm_mday = gps.date.day();
    t.tm_hour = gps.time.hour();
    t.tm_min = gps.time.minute();
    t.tm_sec = gps.time.second();
    t.tm_isdst = 0;
    time_t unix = mktime(&t);
    return (unsigned long)unix;
  }
  return 0;
}

void IRAM_ATTR secondTimer()
{
  unixtime++;
  if(gps_interval_sec == 0)
  {
    gps_getData = true;
    gps_interval_sec = GPS_INTERVAL_SEC;
  }
  gps_interval_sec--;
}

void configSatellite()
{
  satellite_preferences.begin("satellite", false);
  String name = satellite_preferences.getString("name");
  String line1 = satellite_preferences.getString("line1");
  String line2 = satellite_preferences.getString("line2");

  if(name == "" || line1 == "" || line2 == "")
  {
    tle_initialized = false;
    return;
  }

  offset_az = satellite_preferences.getDouble("offset_az");
  offset_el = satellite_preferences.getDouble("offset_el");

  char nameBuf[32], line1Buf[80], line2Buf[80];

  strncpy(nameBuf, name.c_str(), sizeof(nameBuf));
  strncpy(line1Buf, line1.c_str(), sizeof(line1Buf));
  strncpy(line2Buf, line2.c_str(), sizeof(line2Buf));

  nameBuf[sizeof(nameBuf) - 1] = '\0';
  line1Buf[sizeof(line1Buf) - 1] = '\0';
  line2Buf[sizeof(line2Buf) - 1] = '\0';

  satellite.init(nameBuf, line1Buf, line2Buf);
  tle_initialized = true;
}