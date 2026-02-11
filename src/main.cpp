#include "utils.h"
#include "PI_Controller.h"
#include "server.h"
#include "currentSensor.h"
#include "reed.h"
#include "motorDriver.h"
#include "configurations.h"

/* Library Objects */
Sgp4 satellite;
Preferences config;
HardwareSerial gpsSerial(GPS_UART);
TinyGPSPlus gps;
ServerHandler serverHandler(DEFAULT_SERVER_PORT);

/* Discrete Controllers */
PI_Controller controllerAz(KP_AZIMUTH, KI_AZIMUTH, SAMPLE_TIME_S, M_AZ_V_TO_START, M_V_NOMINAL, FREQUENCY_REED_AZ);
PI_Controller controllerEl(KP_ELEVATION, KI_ELEVATION, SAMPLE_TIME_S, M_EL_V_TO_START, M_V_NOMINAL, FREQUENCY_REED_EL);

/* Reed Switch Handlers */
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
TaskHandle_t taskGPS_handle = NULL;
TaskHandle_t taskSGP4Calculation_handle = NULL;
TaskHandle_t taskMotionControl_handle = NULL;
TaskHandle_t taskStopMotion_handle = NULL;

/* ISRs */
void controlTimerISR(void *arg);

/* freeRTOS Tasks*/
void taskGPS(void *pvParameters);
void taskTrackingCalculation(void *pvParameters);
void taskMotionControl(void *pvParameters);
void taskStopMotion(void *pvParameters);
void taskHome(void *pvParameters);
void taskCurrentMonitor(void *pvParameters);

/* Global Variables */
static const char *TAG = "MAIN";
trackerStatus_t status;
esfericalAngles_t target = {0.0, 0.0}, manual_target = {0.0, 0.0}, current = {0.0, 0.0}, offsets_ant = {0.0, 0.0};
bool home_done = false, stopped = false;

void setup()
{
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  Serial.begin(SERIAL_BAUDRATE);

  init_elevation_lut();

  /* Init Calibrations & Configs*/
  currentAz.setShuntValue(SHUNT_AZ_OHM);
  currentEl.setShuntValue(SHUNT_EL_OHM);
  currentAz.setOpAmpGain(CURRENT_GAIN_AZ);
  currentEl.setOpAmpGain(CURRENT_GAIN_EL);
  currentAz.calibrateOffset(CURRENT_OFFSETS_SAMPLES);
  currentEl.calibrateOffset(CURRENT_OFFSETS_SAMPLES);
  ESP_LOGI(TAG, "Shunt off-Az %d [mV], Shunt off-El %d [mV]", currentAz.getOffset_mV(), currentEl.getOffset_mV());
  offsets_ant = getSavedOffsets();
  ESP_LOGI(TAG, "Antenna-Offsets: Az %.3f [°], El %.3f [°]", offsets_ant.azimuth, offsets_ant.elevation);

  controllerAz.setDeadZone(M_AZ_V_TO_START);
  controllerEl.setDeadZone(M_EL_V_TO_START);

  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  status.tle_inited = configSatellite(&satellite);
  configControlTimer(controlTimerISR, SAMPLE_TIME_US);

  // Starting Tasks
  xTaskCreate(taskStopMotion, "Stop Motion Task", 2048, NULL, 17, &taskStopMotion_handle);
  // xTaskCreate(taskCurrentMonitor, "Current Monitor Task", 2048, NULL, 12, NULL);

  serverHandler.start();
}

void loop()
{
  // Update TLE if changed
  if (status.tle_changed == true)
  {
    status.tle_inited = (configSatellite(&satellite) ? true : false);
    status.tle_changed = false;
  }

  // Update Offsets if changed
  if (status.offsets_changed == true)
  {
    offsets_ant = getSavedOffsets();
    status.offsets_changed = false;
    ESP_LOGI(TAG, "Updated Offsets - Az: %.3f [°], El: %.3f [°]", offsets_ant.azimuth, offsets_ant.elevation);
  }

  // GPS Task When no data
  if (status.gps_fix == false && taskGPS_handle == NULL)
  {
    ESP_LOGI(TAG, "Starting GPS Task");
    xTaskCreate(taskGPS, "GPS Task", 2048, NULL, 15, &taskGPS_handle);
  }

  // SGP4 Calculation Task When possible
  if (status.tle_inited == true && status.gps_fix == true && taskSGP4Calculation_handle == NULL)
  {
    ESP_LOGI(TAG, "Starting SGP4 Calculation Task");
    xTaskCreate(taskTrackingCalculation, "Calculation Task", 8192, NULL, 14, &taskSGP4Calculation_handle);
  }

  // Tracking Management
  if (status.tracking == true)
  {
    stopped = false;
    // Home Task
    if (!home_done && taskHome_handle == NULL)
    {
      ESP_LOGI(TAG, "Starting Home Task");
      xTaskCreate(taskHome, "Home Task", 2048, NULL, 16, &taskHome_handle);
    }
    // Home done
    else if (home_done && taskMotionControl_handle == NULL)
    {
      ESP_LOGI(TAG, "Starting Motion Control Task");
      xTaskCreate(taskMotionControl, "Motion Control Task", 8192, NULL, 14, &taskMotionControl_handle);
    }
  }
  // Stopping
  else
  {
    if (!stopped && taskStopMotion_handle == NULL)
    {
      ESP_LOGI(TAG, "Starting Stop Motion Task");
      xTaskCreate(taskStopMotion, "Stop Motion Task", 2048, NULL, 17, &taskStopMotion_handle);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
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

  esfericalAngles_t set_angle = {0.0, 0.0};
  float output_az = 0.0, output_el = 0.0;
  float extension_mm = 0.0, azimut_deg = 0.0;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (status.manual_track == true)
    {
      set_angle.azimuth = manual_target.azimuth;
      set_angle.elevation = manual_target.elevation;
    }
    else
    {
      //Apply Offsets
      set_angle.azimuth = target.azimuth + offsets_ant.azimuth;
      set_angle.elevation = target.elevation + offsets_ant.elevation;
    }

    // Constrain Elevation
    if (set_angle.elevation < 0.0)
    {
      set_angle.elevation = 0.0;
    }
    else if (set_angle.elevation > MAX_ELEVATION_deg)
    {
      set_angle.elevation = MAX_ELEVATION_deg;
    }

    extension_mm = reedEl.getCount() * ELEVATION_RESOLUTION_mm;
    azimut_deg = reedAz.getCount() * AZIMUTH_RESOLUTION_angle;

    current.elevation = elevation_deg_from_mm(extension_mm);
    current.azimuth = azimut_deg;

    output_az = controllerAz.output(azimut_deg, set_angle.azimuth);
    output_el = controllerEl.output(extension_mm, elevation_mm_from_deg(set_angle.elevation));

    if (output_az > 0 && change_forward_az == false)
    {
      change_backward_az = false;
      change_forward_az = true;
      motorAz.setDuty(M_STOP_DUTY);
      motorAz.stop();
      vTaskDelay(pdMS_TO_TICKS(M_AZ_SETTLING_TIME_MS));
      motorAz.setDuty(0);
      motorAz.setDirection(FORWARD);
      reedAz.countDirection(FORWARD);
    }

    if (output_az < 0 && change_backward_az == false)
    {
      change_forward_az = false;
      change_backward_az = true;
      motorAz.setDuty(M_STOP_DUTY);
      motorAz.stop();
      vTaskDelay(pdMS_TO_TICKS(M_AZ_SETTLING_TIME_MS));
      motorAz.setDuty(0);
      motorAz.setDirection(BACKWARD);
      reedAz.countDirection(BACKWARD);
    }

    if (output_el > 0 && change_forward_el == false)
    {
      change_backward_el = false;
      change_forward_el = true;
      motorEl.setDuty(M_STOP_DUTY);
      motorEl.stop();
      vTaskDelay(pdMS_TO_TICKS(M_EL_SETTLING_TIME_MS));
      motorEl.setDuty(0);
      motorEl.setDirection(FORWARD);
      reedEl.countDirection(FORWARD);
    }

    if (output_el < 0 && change_backward_el == false)
    {
      change_forward_el = false;
      change_backward_el = true;
      motorEl.setDuty(M_STOP_DUTY);
      motorEl.stop();
      vTaskDelay(pdMS_TO_TICKS(M_EL_SETTLING_TIME_MS));
      motorEl.setDuty(0);
      motorEl.setDirection(BACKWARD);
      reedEl.countDirection(BACKWARD);
    }

    motorAz.setDuty(fabsf(output_az) * (100.0f / M_V_NOMINAL));
    motorEl.setDuty(fabsf(output_el) * (100.0f / M_V_NOMINAL));
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
          target.azimuth = az[1];
          target.elevation = el[1];
        }
        else
        {
          target.azimuth = az[0] + (az[1] - az[0]) * delta_s;
          target.elevation = el[0] + (el[1] - el[0]) * delta_s;
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

        target.azimuth = az[0];
        target.elevation = el[0];
      }
    }

    vTaskDelay(pdMS_TO_TICKS(SAMPLE_TIME_SGP4_MS));
  }
}

void taskGPS(void *pvParameters)
{
  bool time_set = false;
  for (;;)
  {
    if (status.gps_fix == true)
    {
      vTaskDelete(NULL);
    }

    while (gpsSerial.available() > 0)
    {
      gps.encode(gpsSerial.read());
    }

    if (gps.time.isUpdated() && gps.date.isUpdated() && gps.time.isValid() && gps.date.isValid() && !time_set)
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
      ESP_LOGI(TAG, "GPS Time: %d:%d:%d", gps.time.hour(), gps.time.minute(), gps.time.second());
      time_set = true;
    }

    if (gps.location.isUpdated() && gps.location.isValid() && gps.altitude.isUpdated() && gps.altitude.isValid())
    {
      satellite.site(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
      ESP_LOGI(TAG, "GPS Location: Lat=%.6f, Lng=%.6f, Alt=%.4f", gps.location.lat(), gps.location.lng(), gps.altitude.meters());
      status.gps_fix = true;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void taskHome(void *pvParameters)
{
  uint16_t current_az = 0, current_el = 0;
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
    current_az = currentAz.getCurrent_mA();
    current_el = currentEl.getCurrent_mA();
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

void taskStopMotion(void *pvParameters)
{
  motorAz.setDuty(M_STOP_DUTY);
  motorEl.setDuty(M_STOP_DUTY);
  motorAz.stop();
  motorEl.stop();

  if (taskHome_handle != NULL)
  {
    vTaskDelete(taskHome_handle);
    taskHome_handle = NULL;
    home_done = false;
  }

  if (taskMotionControl_handle != NULL)
  {
    vTaskDelete(taskMotionControl_handle);
    taskMotionControl_handle = NULL;
  }

  vTaskDelay(pdMS_TO_TICKS(200));
  motorAz.setDuty(0);
  motorEl.setDuty(0);

  stopped = true;
  taskStopMotion_handle = NULL;
  vTaskDelete(NULL);
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