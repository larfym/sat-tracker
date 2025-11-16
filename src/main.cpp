#include "utils.h"
#include "pid.h"
#include "server.h"
#include "currentSensor.h"
#include "reed.h"
#include "motorDriver.h"
#include "configurations.h"

static const char *TAG = "MAIN";

Sgp4 satellite;
Preferences config;
HardwareSerial gpsSerial(GPS_UART);
TinyGPSPlus gps;

ServerHandler serverHandler(DEFAULT_SERVER_PORT);
trackerStatus_t status;
antennaPosition_t tar_angle, curr_angle, offsets;

PID pidAz(KP_AZIMUTH, KI_AZIMUTH, KD_AZIMUTH);
PID pidEl(KP_ELEVATION, KI_ELEVATION, KD_ELEVATION);

/* Reed Switch Handlers*/
void IRAM_ATTR reedAz_event_handler(void *arg);
void IRAM_ATTR reedEl_event_handler(void *arg);

/* Components */
ReedSwitch reedAz(REED_AZ_PIN, PCNT_UNIT_0, PCNT_CHANNEL_0, REED_AZ_SOFT_LIMIT_LOW, REED_AZ_SOFT_LIMIT_HIGH, reedAz_event_handler);
ReedSwitch reedEl(REED_EL_PIN, PCNT_UNIT_1, PCNT_CHANNEL_0, REED_EL_SOFT_LIMIT_LOW, REED_EL_SOFT_LIMIT_HIGH, reedEl_event_handler);
MotorDriver motorAz(IN1_AZ_PIN, IN2_AZ_PIN, EN_AZ_PIN, LEDC_TIMER_0, LEDC_CHANNEL_0, PWM_FREQ_HZ);
MotorDriver motorEl(IN1_EL_PIN, IN2_EL_PIN, EN_EL_PIN, LEDC_TIMER_0, LEDC_CHANNEL_1, PWM_FREQ_HZ);
CurrentSensor currentAz(I_AZ_CHANNEL);
CurrentSensor currentEl(I_EL_CHANNEL);

TaskHandle_t taskTracking_handle = NULL;

void controlTimerISR(void *arg);

/* freeRTOS Tasks*/
void taskGPS(void *pvParameters);
void taskTracking(void *pvParameters);
void taskHome(void *pvParameters);
void taskCurrentMonitor(void *pvParameters);

void setup()
{
  esp_log_level_set("*", ESP_LOG_INFO);
  Serial.begin(SERIAL_BAUDRATE);
  init_elevation_lut();
  
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  status.tle_inited = configSatellite(&satellite);
  configControlTimer(controlTimerISR, SAMPLE_TIME_US);

  xTaskCreate(taskGPS, "GPS Task", 2048, NULL, 10, NULL);
  xTaskCreate(taskTracking, "TrackingTask", 8192, NULL, 15, &taskTracking_handle);

  ESP_LOGI(TAG, "SETUP");
  serverHandler.start();
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1500));

  if (status.tle_changed == true)
  {
    if (configSatellite(&satellite))
    {
      status.tle_inited = true;
    }
    else
    {
      status.tle_inited = false;
    }
    status.tle_changed = false;
  }
}

void IRAM_ATTR controlTimerISR(void *arg)
{
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (status.tracking == true && taskTracking_handle != NULL)
  {
    vTaskNotifyGiveFromISR(taskTracking_handle, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void taskTracking(void *pvParameters)
{
  TickType_t time_ms_ref = 0;
  unsigned long last_unixTime = 0;
  double az[2], el[2];
  bool first_run = true;
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //TODO Verificar el estado HOME
    //TODO Interpol_time =/ SAMPLE_TIME_US
    if (!status.manual_track)
    {
      unsigned long current_time_ms = millis();
      unsigned long current_unixTime = time(NULL);

      if (current_unixTime == last_unixTime)
      {
        // Interpolación Lineal
        double delta_s = (current_time_ms - time_ms_ref) / 1000.0;

        if (delta_s >= 1.0)
        {
          tar_angle.azimuth = az[1];
          tar_angle.elevation = el[1];
        }
        else
        {
          tar_angle.azimuth = az[0] + (az[1] - az[0]) * delta_s;
          tar_angle.elevation = el[0] + (el[1] - el[0]) * delta_s;
        }
      }
      else
      {
        if (first_run == true)
        {
          first_run = false;
          satellite.findsat(current_unixTime);
          az[0] = satellite.satAz;
          el[0] = satellite.satEl;

          satellite.findsat(current_unixTime + 1);
          az[1] = satellite.satAz;
          el[1] = satellite.satEl;
        }
        else
        {
          az[0] = az[1];
          el[0] = el[1];

          satellite.findsat(current_unixTime + 1);
          az[1] = satellite.satAz;
          el[1] = satellite.satEl;
        }

        last_unixTime = current_unixTime;
        time_ms_ref = current_time_ms;

        tar_angle.azimuth = az[0];
        tar_angle.elevation = el[0];
      }
    }

    // TODO PID Control here USING tar_angle
    pidAz.setPoint(tar_angle.azimuth);
    pidEl.setPoint(tar_angle.elevation);
    
  }
}

void taskGPS(void *pvParameters)
{
  for (;;)
  {
    while (gpsSerial.available() > 0)
    {
      gps.encode(gpsSerial.read());
    }

    if (gps.time.isUpdated() && gps.date.isUpdated() && gps.time.isValid() && gps.date.isValid())
    {
      struct tm t;
      t.tm_year = gps.date.year() - 1900;
      t.tm_mon = gps.date.month() - 1;
      t.tm_mday = gps.date.day();
      t.tm_hour = gps.time.hour();
      t.tm_min = gps.time.minute();
      t.tm_sec = gps.time.second();

      time_t tim = mktime(&t);
      struct timeval unix;
      unix.tv_sec = tim;
      unix.tv_usec = 0;
      settimeofday(&unix, NULL);
    }

    if (gps.location.isUpdated() && gps.location.isValid() && gps.altitude.isUpdated())
    {
      satellite.site(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
      ESP_LOGI(TAG, "GPS Fix: Lat=%.6f, Lng=%.6f, Alt=%.4f", gps.location.lat(), gps.location.lng(), gps.altitude.meters());
      ESP_LOGI(TAG, "Time Now: %d:%d", gps.time.minute(), gps.time.second());

      if (status.gps_fix == false)
      {
        status.gps_fix = true;
        vTaskDelete(NULL);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void taskHome(void *pvParameters)
{
  reedAz.stopCount();
  reedEl.stopCount();
  motorAz.setDuty(30);
  motorEl.setDuty(30);
  motorAz.stop();
  motorEl.stop();

  vTaskDelay(pdMS_TO_TICKS(500));
  motorAz.setDuty(100);
  motorEl.setDuty(100);
  motorAz.setDirection(FORWARD);
  motorEl.setDirection(FORWARD);
  vTaskDelay(pdMS_TO_TICKS(1500));

  motorAz.stop();
  motorEl.stop();
  motorAz.setDirection(BACKWARD);
  motorEl.setDirection(BACKWARD);
  vTaskDelay(pdMS_TO_TICKS(1000));

  for (;;)
  {
    uint16_t current_az = currentAz.getCurrent_mA();
    uint16_t current_el = currentEl.getCurrent_mA();
    if (current_az < CURRENT_HOMING_mA && current_el < CURRENT_HOMING_mA)
    {
      motorAz.stop();
      motorEl.stop();
      motorAz.setDuty(0);
      motorEl.setDuty(0);
      motorAz.setDirection(FORWARD);
      motorEl.setDirection(FORWARD);

      reedAz.resetCount();
      reedEl.resetCount();
      reedAz.startCount();
      reedEl.startCount();
      ESP_LOGI(TAG, "Home Reached");
      vTaskDelete(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// TODO Ignore motor peak current at start up
void taskCurrentMonitor(void *pvParameters)
{
  for (;;)
  {
    float current_az = currentAz.getCurrent_mA();
    float current_el = currentEl.getCurrent_mA();

    if (current_az > CURRENT_AZIMUTH_MAX_mA)
    {
      ESP_LOGW(TAG, "Azimuth Motor Overcurrent: %.2f mA", current_az);
    }
    if (current_el > CURRENT_ELEVATION_MAX_mA)
    {
      ESP_LOGW(TAG, "Elevation Motor Overcurrent: %.2f mA", current_el);
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void IRAM_ATTR reedAz_event_handler(void *arg)
{
  // TODO Software endstops
}

void IRAM_ATTR reedEl_event_handler(void *arg)
{
  // TODO Software endstops
}