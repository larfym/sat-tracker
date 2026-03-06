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
PI_Controller controllerAz(KP_AZIMUTH, KI_AZIMUTH, SAMPLE_TIME_S, M_AZ_V_TO_START, M_V_NOMINAL);
PI_Controller controllerEl(KP_ELEVATION, KI_ELEVATION, SAMPLE_TIME_S, M_EL_V_TO_START, M_V_NOMINAL);

/* Reed Switch Handlers */
void IRAM_ATTR reedAz_event_handler(void *arg);
void IRAM_ATTR reedEl_event_handler(void *arg);

/* ISRs */
void controlTimer_ISR(void *arg);

/* Components */
ReedSwitch reedAz(REED_AZ_PIN, PCNT_UNIT_0, PCNT_CHANNEL_0, REED_AZ_SOFT_LIMIT_LOW, REED_AZ_SOFT_LIMIT_HIGH, reedAz_event_handler);
ReedSwitch reedEl(REED_EL_PIN, PCNT_UNIT_1, PCNT_CHANNEL_0, REED_EL_SOFT_LIMIT_LOW, REED_EL_SOFT_LIMIT_HIGH, reedEl_event_handler);
MotorDriver motorAz(IN1_AZ_PIN, IN2_AZ_PIN, EN_AZ_PIN, LEDC_TIMER_0, LEDC_CHANNEL_0, PWM_FREQ_HZ);
MotorDriver motorEl(IN1_EL_PIN, IN2_EL_PIN, EN_EL_PIN, LEDC_TIMER_0, LEDC_CHANNEL_1, PWM_FREQ_HZ);
CurrentSensor currentAz(I_AZ_CHANNEL);
CurrentSensor currentEl(I_EL_CHANNEL);

/* Task Handlers */
TaskHandle_t GPS_TaskHandle = NULL;
TaskHandle_t TrackingPredictor_TaskHandle = NULL;
TaskHandle_t MotionControl_TaskHandle = NULL;
TaskHandle_t StopMotion_TaskHandle = NULL;
TaskHandle_t Calibration_TaskHandle = NULL;

/* freeRTOS Tasks */
void GPS_Task(void *pvParameters);
void TrackingPredictor_Task(void *pvParameters);
void MotionControl_Task(void *pvParameters);
void StopMotion_Task(void *pvParameters);
void Calibration_Task(void *pvParameters);
void CurrentMonitor_Task(void *pvParameters);
void telemetry_Task(void *pvParameters);

/* Global Variables */
static const char *TAG = "MAIN";
esfericalAngles_t target = {0};
esfericalAngles_t manual_target = {0};
esfericalAngles_t current = {0};
esfericalAngles_t offsets_ant = {0};
esfericalAngles_t set_angle = {0};
mountFlags_t flags;
trackerStatus_t status;
unsigned long next_pass_unix = 0;

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
  ESP_LOGI(TAG, "Shunt-Voltage-Offsets: Az %d [mV], El %d [mV]", currentAz.getOffset_mV(), currentEl.getOffset_mV());
  offsets_ant = getSavedOffsets();
  ESP_LOGI(TAG, "Antenna-Offsets: Az %.3f [°], El %.3f [°]", offsets_ant.azimuth, offsets_ant.elevation);

  controllerAz.setDeadZone(M_AZ_V_TO_START);
  controllerEl.setDeadZone(M_EL_V_TO_START);

  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  status.tle = configSatellite(&satellite);
  configControlTimer(controlTimer_ISR, SAMPLE_TIME_US);

  // Starting Tasks
  xTaskCreate(StopMotion_Task, "Stop Motion Task", MEDIUM_STACK_SIZE, NULL, TASK_STOP_MOTION_PRIORITY, &StopMotion_TaskHandle);
  xTaskCreate(telemetry_Task, "Telemetry Task", MAX_STACK_SIZE, NULL, TASK_TELEMETRY_PRIORITY, NULL);
  // xTaskCreate(CurrentMonitor_Task, "Current Monitor Task", MEDIUM_STACK_SIZE, NULL, 12, NULL);

  serverHandler.start();
}

void loop()
{
  // Reset
  if (flags.reset_pending == true)
  {
    ESP_LOGI(TAG, "Restarting...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP.restart();
  }

  // Update TLE if changed
  if (flags.tle_updated == true)
  {
    status.tle = (configSatellite(&satellite) ? true : false);
    flags.tle_updated = false;
    flags.pred_done = false;
    next_pass_unix = 0;
  }

  // Update Offsets if changed
  if (flags.offsets_updated == true)
  {
    offsets_ant = getSavedOffsets();
    flags.offsets_updated = false;
    flags.pred_done = false;
    next_pass_unix = 0;
  }

  // GPS Task When no data
  if (status.gps == false && GPS_TaskHandle == NULL)
  {
    ESP_LOGI(TAG, "Starting GPS Task");
    xTaskCreate(GPS_Task, "GPS Task", MEDIUM_STACK_SIZE, NULL, TASK_GPS_PRIORITY, &GPS_TaskHandle);
  }

  // SGP4 Calculation Task When possible
  if (status.tle == true && status.gps == true && TrackingPredictor_TaskHandle == NULL)
  {
    ESP_LOGI(TAG, "Starting Predictor Task");
    xTaskCreate(TrackingPredictor_Task, "Predictor Task", MAX_STACK_SIZE, NULL, TASK_TRACKING_CALCULATION_PRIORITY, &TrackingPredictor_TaskHandle);
  }

  // Tracking Management
  if (status.tracking == true)
  {
    flags.stop_done = false;
    // Calibration Task
    if (!flags.home_done && Calibration_TaskHandle == NULL)
    {
      ESP_LOGI(TAG, "Starting Calibration Task");
      xTaskCreate(Calibration_Task, "Calibration", MEDIUM_STACK_SIZE, NULL, TASK_HOME_PRIORITY, &Calibration_TaskHandle);
    }
    else if (flags.home_done && MotionControl_TaskHandle == NULL)
    {
      ESP_LOGI(TAG, "Starting Motion Control Task");
      xTaskCreate(MotionControl_Task, "Motion Control", MAX_STACK_SIZE, NULL, TASK_MOTION_CONTROL_PRIORITY, &MotionControl_TaskHandle);
    }
  }
  // Stopping
  else
  {
    if (!flags.stop_done && StopMotion_TaskHandle == NULL)
    {
      ESP_LOGI(TAG, "Starting Stop Motion Task");
      xTaskCreate(StopMotion_Task, "Stop Motion", MEDIUM_STACK_SIZE, NULL, TASK_STOP_MOTION_PRIORITY, &StopMotion_TaskHandle);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(TASK_LOOP_DELAY));
}

// SampleTime ISR for Motion Control Task
void IRAM_ATTR controlTimer_ISR(void *arg)
{
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (status.tracking == true && MotionControl_TaskHandle != NULL)
  {
    vTaskNotifyGiveFromISR(MotionControl_TaskHandle, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// PI & Movement
void MotionControl_Task(void *pvParameters)
{
  constexpr float DUTY_SCALE = 100.0f / M_V_NOMINAL;
  const uint8_t search_iterations = 30;

  float el_mm = 0.0f;
  float output_az = 0.0, output_el = 0.0;
  int new_dir_az = 0, new_dir_el = 0, current_dir_az = 0, current_dir_el = 0;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // SETPOINT Calculation with Offsets
    if (status.manual_tracking == true)
    {
      set_angle.azimuth = manual_target.azimuth;
      set_angle.elevation = manual_target.elevation;
    }
    else
    {
      // Below Horizon - Next Pass Prediction
      if (target.elevation < offsets_ant.elevation)
      {
        if (!flags.pred_done)
        {
          passinfo next_pass;
          satellite.initpredpoint((unsigned long)time(NULL), offsets_ant.elevation);
          if (satellite.nextpass(&next_pass, search_iterations, false, offsets_ant.elevation))
          {
            next_pass_unix = jdToUnix(next_pass.jdstart);
            set_angle.azimuth = next_pass.azstart - offsets_ant.azimuth;
            set_angle.elevation = EL_MIN_DEG;
            ESP_LOGI(TAG, "Next Pass - Azimuth: %.2f [°], Elevation: %.2f [°]", set_angle.azimuth, set_angle.elevation);
          }
          else
          {
            next_pass_unix = 0;
            ESP_LOGI(TAG, "No upcoming pass found above the horizon.");
            set_angle.azimuth = AZ_MIN_DEG;
            set_angle.elevation = EL_MAX_DEG;
          }
          flags.pred_done = true;
        }
      }
      else
      {
        next_pass_unix = 0;
        flags.pred_done = false;
        set_angle.azimuth = target.azimuth - offsets_ant.azimuth;
        set_angle.elevation = target.elevation - offsets_ant.elevation;
      }
    }

    // ANGLE SETPOINT LIMITS
    set_angle.azimuth = fmaxf(fminf(set_angle.azimuth, AZ_MAX_DEG), AZ_MIN_DEG);
    set_angle.elevation = fmaxf(fminf(set_angle.elevation, EL_MAX_DEG), EL_MIN_DEG);

    el_mm = reedEl.getCount() * ELEVATION_RESOLUTION_mm;
    current.elevation = elevation_deg_from_mm(el_mm);
    current.azimuth = reedAz.getCount() * AZIMUTH_RESOLUTION_angle;

    // PID Controller
    output_az = controllerAz.output(current.azimuth, set_angle.azimuth);
    output_el = controllerEl.output(el_mm, elevation_mm_from_deg(set_angle.elevation));

    // CONTROL ACTION
    new_dir_az = (output_az > 0) ? FORWARD : (output_az < 0) ? BACKWARD
                                                             : current_dir_az;
    new_dir_el = (output_el > 0) ? FORWARD : (output_el < 0) ? BACKWARD
                                                             : current_dir_el;

    // Azimut
    if (new_dir_az != current_dir_az)
    {
      controllerAz.reset();
      motorAz.stop();
      vTaskDelay(pdMS_TO_TICKS(M_AZ_SETTLING_TIME_MS));
      motorAz.setDuty(0);
      motorAz.setDirection((direction)new_dir_az);
      reedAz.countDirection((direction)new_dir_az);
      current_dir_az = new_dir_az;
    }

    // Elevation
    if (new_dir_el != current_dir_el)
    {
      controllerEl.reset();
      motorEl.stop();
      vTaskDelay(pdMS_TO_TICKS(M_EL_SETTLING_TIME_MS));
      motorEl.setDuty(0);
      motorEl.setDirection((direction)new_dir_el);
      reedEl.countDirection((direction)new_dir_el);
      current_dir_el = new_dir_el;
    }

    motorAz.setDuty(fabsf(output_az) * DUTY_SCALE);
    motorEl.setDuty(fabsf(output_el) * DUTY_SCALE);
  }
}

// Calculate SGP4 & Interpolations
void TrackingPredictor_Task(void *pvParameters)
{
  TickType_t time_ms_ref = 0;
  unsigned long last_unixTime = 0;
  double az[2], el[2];
  bool first_run = true;
  for (;;)
  {

    if (!status.manual_tracking)
    {
      unsigned long current_time_ms = millis();
      unsigned long current_unixTime = time(NULL);

      if (current_unixTime == last_unixTime)
      {
        // Lineal Interpolation
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

void GPS_Task(void *pvParameters)
{
  bool time_set = false;
  for (;;)
  {
    if (status.gps == true)
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

      setTime(tim);
      time_set = true;
    }

    if (gps.location.isUpdated() && gps.location.isValid() && gps.altitude.isUpdated() && gps.altitude.isValid())
    {
      satellite.site(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
      status.gps = true;
    }
    vTaskDelay(pdMS_TO_TICKS(TASK_GPS_DELAY));
  }
}

void Calibration_Task(void *pvParameters)
{
  uint16_t current_az = 0, current_el = 0;

  reedAz.stopCount();
  reedEl.stopCount();

  motorAz.stop();
  motorEl.stop();
  vTaskDelay(pdMS_TO_TICKS(100));

  motorAz.setDuty(M_MAX_DUTY);
  motorEl.setDuty(M_MAX_DUTY);
  motorAz.setDirection(FORWARD);
  motorEl.setDirection(FORWARD);
  vTaskDelay(pdMS_TO_TICKS(500));

  motorAz.stop();
  motorEl.stop();
  vTaskDelay(pdMS_TO_TICKS(100));

  motorAz.setDuty(M_MAX_DUTY);
  motorEl.setDuty(M_MAX_DUTY);
  motorAz.setDirection(BACKWARD);
  motorEl.setDirection(BACKWARD);
  vTaskDelay(pdMS_TO_TICKS(300));

  for (;;)
  {
    current_az = currentAz.getCurrent_mA();
    current_el = currentEl.getCurrent_mA();
    if (current_az < CURRENT_HOMING_mA && current_el < CURRENT_HOMING_mA)
    {
      motorAz.stop();
      motorEl.stop();
      vTaskDelay(pdMS_TO_TICKS(100));

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
      flags.home_done = true;
      Calibration_TaskHandle = NULL;
      vTaskDelete(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(TASK_HOME_CURRENT_DELAY));
  }
}

void StopMotion_Task(void *pvParameters)
{

  if (Calibration_TaskHandle != NULL)
  {
    vTaskDelete(Calibration_TaskHandle);
    Calibration_TaskHandle = NULL;
    flags.home_done = false;
  }

  if (MotionControl_TaskHandle != NULL)
  {
    vTaskDelete(MotionControl_TaskHandle);
    MotionControl_TaskHandle = NULL;
  }

  motorAz.stop();
  motorEl.stop();
  vTaskDelay(pdMS_TO_TICKS(100));
  motorAz.setDuty(0);
  motorEl.setDuty(0);

  motorAz.setDirection(FORWARD);
  motorEl.setDirection(FORWARD);
  reedAz.countDirection(FORWARD);
  reedEl.countDirection(FORWARD);

  flags.stop_done = true;
  StopMotion_TaskHandle = NULL;
  vTaskDelete(NULL);
}

// TODO: Ignore motor peak current at start up
void CurrentMonitor_Task(void *pvParameters)
{
  for (;;)
  {
    float current_az = currentAz.getCurrent_mA();
    float current_el = currentEl.getCurrent_mA();

    if (current_az > CURRENT_AZIMUTH_MAX_mA)
    {
      ESP_LOGW(TAG, "Azimuth Motor Overcurrent: %.2f mA", current_az);
      status.tracking = false;
      status.error = TRACKER_ERROR_OVERCURRENT_AZ;
    }
    if (current_el > CURRENT_ELEVATION_MAX_mA)
    {
      ESP_LOGW(TAG, "Elevation Motor Overcurrent: %.2f mA", current_el);
      status.tracking = false;
      status.error = TRACKER_ERROR_OVERCURRENT_EL;
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_CURRENT_MONITOR_DELAY));
  }
}

void telemetry_Task(void *pvParameters)
{
  for (;;)
  {
    serverHandler.sendTelemetry();
    vTaskDelay(pdMS_TO_TICKS(TASK_TELEMETRY_DELAY));
  }
}

void IRAM_ATTR reedAz_event_handler(void *arg)
{
  status.tracking = false;
  status.error = TRACKER_ERROR_SOFT_ENDSTOP;
  // TODO Software endstops
}

void IRAM_ATTR reedEl_event_handler(void *arg)
{
  status.tracking = false;
  status.error = TRACKER_ERROR_SOFT_ENDSTOP;
  // TODO Software endstops
}