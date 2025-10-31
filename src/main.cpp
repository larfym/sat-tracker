#include "utils.h"
#include "server.h"
#include "currentMeasure.h"
#include "reed.h"
#include "motorDriver.h"
#include "pid.h"
#include "pins.h"

static const char *TAG = "MAIN";

unsigned long unixtime;

Sgp4 satellite;
Preferences config;
HardwareSerial gpsSerial(GPS_UART);
TinyGPSPlus gps;

ServerHandler serverHandler(DEFAULT_SERVER_PORT);
trackerStatus_t status;
antennaPosition_t tar_angle, curr_angle, offsets;

void IRAM_ATTR reed_az_event_handler(void *arg);
void IRAM_ATTR reed_el_event_handler(void *arg);

/* Components*/
MotorDriver motor_az(IN1_AZ, IN2_AZ, EN_AZ, LEDC_TIMER_0, LEDC_CHANNEL_0, PWM_FREQ_HZ);
MotorDriver motor_el(IN1_EL, IN2_EL, EN_EL, LEDC_TIMER_0, LEDC_CHANNEL_1, PWM_FREQ_HZ);
CurrentSensor i_el(ADC1_CHANNEL_0);
CurrentSensor i_az(ADC1_CHANNEL_1);
ReedSwitch reed_az(REED_AZ, PCNT_UNIT_0, PCNT_CHANNEL_0, -1, 1850, reed_az_event_handler);
ReedSwitch reed_el(REED_EL, PCNT_UNIT_0, PCNT_CHANNEL_1, -1, 600, reed_el_event_handler);

TaskHandle_t taskTracking_handle = NULL;

void secondTimerISR(void *arg);

void taskGPS(void *pvParameters);
void taskTracking(void *pvParameters);
void taskHome(void *pvParameters);

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    esp_log_level_set("*", ESP_LOG_INFO);

    gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    status.tle_inited = configSatellite(&satellite);
    configSecondTimer(secondTimerISR);

    xTaskCreate(taskGPS, "GPS Task", 2048, NULL, 10, NULL);
    xTaskCreate(taskTracking, "TrackingTask", 8192, NULL, 15, &taskTracking_handle);

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

void IRAM_ATTR secondTimerISR(void *arg)
{
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);

    unixtime++;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (taskTracking_handle != NULL)
    {
        vTaskNotifyGiveFromISR(taskTracking_handle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void taskTracking(void *pvParameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        satellite.findsat(unixtime);
        tar_angle.azimuth = satellite.satAz;
        tar_angle.elevation = satellite.satEl;
    }
}

// TODO fix serial buffer update
void taskGPS(void *pvParameters)
{
    const TickType_t updatePeriodFix = pdMS_TO_TICKS(60000);
    const TickType_t updatePeriodNoFix = pdMS_TO_TICKS(1000);

    for (;;)
    {

        while (gpsSerial.available() > 0)
        {
            gps.encode(gpsSerial.read());
        }

        if (gps.time.isUpdated() && gps.date.isUpdated() && gps.time.isValid() && gps.date.isValid())
        {
            unixtime = getUnixTimeFromGPS(&gps);
            ESP_LOGI(TAG, "Tiempo actualizado: %d:%d", gps.time.minute(), gps.time.second());
        }

        if (gps.location.isUpdated() && gps.location.isValid())
        {
            satellite.site(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
            ESP_LOGI(TAG, "GPS adquirido: Lat=%.6f, Lng=%.6f", gps.location.lat(), gps.location.lng());

            if (status.gps_fix == false)
            {
                status.gps_fix = true;
                TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();
                vTaskPrioritySet(currentTaskHandle, 2);
            }
        }

        vTaskDelay(status.gps_fix ? updatePeriodFix : updatePeriodNoFix);
    }
}

void taskHome(void *pvParameters)
{
    reed_az.stopCount();
    reed_el.stopCount();
    motor_az.setDuty(30);
    motor_el.setDuty(30);
    motor_az.stop();
    motor_el.stop();

    vTaskDelay(pdMS_TO_TICKS(500));
    motor_az.setDuty(100);
    motor_el.setDuty(100);
    motor_az.setDirection(FORWARD);
    motor_el.setDirection(FORWARD);
    vTaskDelay(pdMS_TO_TICKS(1500));

    motor_az.stop();
    motor_el.stop();
    motor_az.setDirection(BACKWARD);
    motor_el.setDirection(BACKWARD);
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (;;)
    {
        uint16_t current_az = i_az.getCurrent_mA();
        uint16_t current_el = i_el.getCurrent_mA();
        if(current_az < 150 && current_el < 150)
        {   
            motor_az.stop();
            motor_el.stop();
            motor_az.setDuty(0);
            motor_el.setDuty(0);
            motor_az.setDirection(FORWARD);
            motor_el.setDirection(FORWARD);

            reed_az.resetCount();
            reed_el.resetCount();
            reed_az.startCount();
            reed_el.startCount();
            ESP_LOGI(TAG, "Home Reached");
            vTaskDelete(NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void IRAM_ATTR reed_az_event_handler(void *arg)
{
    //TODO Software endstops
}

void IRAM_ATTR reed_el_event_handler(void *arg)
{
    //TODO Software endstops
}