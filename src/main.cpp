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
antennaPosition_t tar_angle, curr_angle, offsets_angle;

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

/* Task Handlers */
TaskHandle_t taskHome_handle = NULL;
TaskHandle_t taskTrackingCalculation_handle = NULL;
TaskHandle_t taskMotionControl_handle = NULL;

/* ISRs */
void controlTimerISR(void *arg);

/* freeRTOS Tasks*/
void taskGPS(void *pvParameters);
void taskTrackingCalculation(void *pvParameters);
void taskMotionControl(void *pvParameters);
void taskHome(void *pvParameters);
void taskCurrentMonitor(void *pvParameters);

/* Global Variables */
bool home_done = false;

void setup()
{
  esp_log_level_set("*", ESP_LOG_INFO);
  Serial.begin(SERIAL_BAUDRATE);
  init_elevation_lut();
  currentAz.calibrateOffset(CURRENT_OFFSETS_SAMPLES);
  currentEl.calibrateOffset(CURRENT_OFFSETS_SAMPLES);
  ESP_LOGI(TAG, "Current offset Az %d [mV], Current offset El %d [mV]", currentAz.getOffset_mV(), currentEl.getOffset_mV());
  offsets_angle = getSavedOffsets();
  ESP_LOGI(TAG, "Saved offset Az %.3f [°], Saved offset El %.3f [°]", offsets_angle.azimuth, offsets_angle.elevation);
  
  motorAz.setMotorLowerBound(M_AZ_DUTY_TO_START);
  motorEl.setMotorLowerBound(M_EL_DUTY_TO_START);
  pidAz.setConstraintsAtIntegralError(-INTEGRAL_CONSTRAIN_AZ, INTEGRAL_CONSTRAIN_AZ);
  pidEl.setConstraintsAtIntegralError(-INTEGRAL_CONSTRAIN_EL, INTEGRAL_CONSTRAIN_EL);

  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  status.tle_inited = configSatellite(&satellite);
  configControlTimer(controlTimerISR, SAMPLE_TIME_US);

  //Starting Tasks
  xTaskCreate(taskGPS, "GPS Task", 2048, NULL, 16, NULL);
  //xTaskCreate(taskCurrentMonitor, "Current Monitor Task", 2048, NULL, 12, NULL);

  serverHandler.start();
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1500));

  if (status.tle_changed == true)
  {
    status.tle_inited = (configSatellite(&satellite)? true : false);
    status.tle_changed = false;
  }

  //Home Task When needed
  if(status.tracking == true)
  {
    if(!home_done)
    {
      if(taskHome_handle == NULL)
      {
        ESP_LOGI(TAG, "Starting Home Task");
        xTaskCreate(taskHome, "Home Task", 2048, NULL, 19, &taskHome_handle);
      }
    }else //Home done
    { 
      if(!status.manual_track && taskTrackingCalculation_handle == NULL)
      {
        xTaskCreate(taskTrackingCalculation, "Tracking Task", 8192, NULL, 18, &taskTrackingCalculation_handle);
      }
      if(taskMotionControl_handle == NULL)
      {
        ESP_LOGI(TAG, "Starting Motion Task");
        xTaskCreate(taskMotionControl, "Motion Control Task", 2048, NULL, 20, &taskMotionControl_handle);
      }
    }
  }else
  {
    home_done = false;
    if (taskMotionControl_handle != NULL) { vTaskDelete(taskMotionControl_handle); taskMotionControl_handle = NULL; }
    if (taskTrackingCalculation_handle != NULL) { vTaskDelete(taskTrackingCalculation_handle); taskTrackingCalculation_handle = NULL; }
    if (taskHome_handle != NULL) { vTaskDelete(taskHome_handle); taskHome_handle = NULL; } 
  }

}

void IRAM_ATTR controlTimerISR(void *arg)
{
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (status.tracking == true && taskMotionControl_handle != NULL)
  {
    vTaskNotifyGiveFromISR(taskMotionControl_handle, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// PID & Movement
void taskMotionControl(void *pvParameters)
{
  bool change_forward_az = false, change_backward_az = false;
  bool change_forward_el = false, change_backward_el = false;
  float tar_elevation = 0.0;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!status.tracking)
    {
      taskMotionControl_handle = NULL;
      vTaskDelete(NULL);
    }
    if (!home_done)
      continue;

    //Constrain Elevation angle
    if(tar_angle.elevation > MAX_ELEVATION_deg)
    {
      tar_elevation = MAX_ELEVATION_deg;
    }else if(tar_angle.elevation < 0.0)
    {
      tar_elevation = 0.0;
    }else{
      tar_elevation = tar_angle.elevation;
    }

    float extension_mm = reedEl.getCount()*ELEVATION_RESOLUTION_mm;
    float azimut_deg = reedAz.getCount()*AZIMUTH_RESOLUTION_angle;

    curr_angle.elevation = elevation_deg_from_mm(extension_mm);
    curr_angle.azimuth = azimut_deg;
    ESP_LOGI(TAG, "Az angl: %0.2f, El mm: %0.2f, El deg: %0.2f", curr_angle.azimuth, extension_mm, curr_angle.elevation);

    pidEl.setPoint(elevation_mm_from_deg(tar_elevation));
    pidAz.setPoint(tar_angle.azimuth);

    double o_az = pidAz.output(curr_angle.azimuth);
    double o_el = pidEl.output(extension_mm);

    if(o_az > 0 && change_forward_az == false)
    {
      change_backward_az = false;
      change_forward_az = true;
      motorAz.stop();
      vTaskDelay(pdMS_TO_TICKS(100));
      motorAz.setDuty(0);
      motorAz.setDirection(FORWARD);
      reedAz.countDirection(FORWARD);
    }

    if(o_az < 0 && change_backward_az == false)
    {
      change_forward_az = false;
      change_backward_az = true;
      motorAz.stop();
      vTaskDelay(pdMS_TO_TICKS(100));
      motorAz.setDuty(0);
      motorAz.setDirection(BACKWARD);
      reedAz.countDirection(BACKWARD);
    }

    if(o_el > 0 && change_forward_el == false)
    {
      change_backward_el = false;
      change_forward_el = true;
      motorEl.stop();
      vTaskDelay(pdMS_TO_TICKS(100));
      motorEl.setDuty(0);
      motorEl.setDirection(FORWARD);
      reedEl.countDirection(FORWARD);
    }

    if(o_el < 0 && change_backward_el == false)
    {
      change_forward_az = false;
      change_backward_el = true;
      motorEl.stop();
      vTaskDelay(pdMS_TO_TICKS(100));
      motorEl.setDuty(0);
      motorEl.setDirection(BACKWARD);
      reedEl.countDirection(BACKWARD);
    }

    ESP_LOGI(TAG, "o_az: %0.2f, o_el: %0.2f", o_az, o_el);

    motorAz.setMotorDuty(abs(o_az));
    motorAz.setMotorDuty(abs(o_el));
  }
}

// Calculate SGP4 & Interpolations
void taskTrackingCalculation(void *pvParameters)
{
  TickType_t time_ms_ref = 0;
  unsigned long last_unixTime = 0;
  double az[2], el[2];
  bool first_run = true;
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!status.tracking)
    {
      taskTrackingCalculation_handle = NULL;
      vTaskDelete(NULL);
    }

    if (!status.manual_track)
    {
      unsigned long current_time_ms = millis();
      unsigned long current_unixTime = time(NULL);

      if (current_unixTime == last_unixTime)
      {
        // Interpolación Lineal de Azimuth y Elevación
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
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_TIME_SGP4_MS));
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

    if (gps.location.isUpdated() && gps.location.isValid() && gps.altitude.isUpdated() && gps.altitude.isValid())
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
    ESP_LOGI(TAG, "Curr_az %d, Curr_el %d", current_az, current_el);
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
      reedAz.countDirection(FORWARD);
      reedEl.countDirection(FORWARD);
      reedAz.startCount();
      reedEl.startCount();
      ESP_LOGI(TAG, "Home Reached");
      home_done = true;
      taskHome_handle = NULL;
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