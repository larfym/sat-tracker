#include "utils.h"

static const char *TAG = "UTILS";

bool configSatellite(Sgp4 *satellite)
{
    config.begin("config", false);
    String name = config.getString("name", "");
    String line1 = config.getString("line1", "");
    String line2 = config.getString("line2", "");

    config.end();

    ESP_LOGI(TAG, "Loaded TLE Data -> %s: tle1: %s, tle2: %s, offset_az: %f, offset_el: %f\n", name.c_str(), line1.c_str(), line2.c_str(), offset_az, offset_el);
    if (name.length() == 0 || line1.length() == 0 || line2.length() == 0)
    {
        ESP_LOGI(TAG, "Error configuring satellite TLE");
        return false;
    }

    char nameBuf[32], line1Buf[80], line2Buf[80];

    strlcpy(nameBuf, name.c_str(), sizeof(nameBuf));
    strlcpy(line1Buf, line1.c_str(), sizeof(line1Buf));
    strlcpy(line2Buf, line2.c_str(), sizeof(line2Buf));

    satellite->init(nameBuf, line1Buf, line2Buf);
    ESP_LOGI(TAG, "Satellite TLE data configured");
    return true;
}

antennaPosition_t getSavedOffsets(void)
{
    antennaPosition_t offsets;
    config.begin("config", false);
    offsets.azimuth = config.getDouble("offset_az", 0.0);
    offsets.elevation = config.getDouble("offset_el", 0.0);
    config.end();
    return offsets;
}

void saveTLEdata(String name, String line1, String line2, double offset_az, double offset_el)
{
    config.begin("config", false);
    config.putString("name", name);
    config.putString("line1", line1);
    config.putString("line2", line2);
    config.putDouble("offset_az", offset_az);
    config.putDouble("offset_el", offset_el);
    config.end();
    ESP_LOGI(TAG, "Data Saved - Name: %s tle1: %s, tle2: %s, offset_az: %f, offset_el: %f\n", name.c_str(), line1.c_str(), line2.c_str(), offset_az, offset_el);
}

unsigned long getUnixTimeFromGPS(TinyGPSPlus *gps)
{
    if (gps->date.isValid() && gps->time.isValid())
    {
        struct tm t;
        t.tm_year = gps->date.year() - 1900;
        t.tm_mon = gps->date.month() - 1;
        t.tm_mday = gps->date.day();
        t.tm_hour = gps->time.hour();
        t.tm_min = gps->time.minute();
        t.tm_sec = gps->time.second();
        t.tm_isdst = 0;
        time_t unix = mktime(&t);
        return (unsigned long)unix;
    }
    return 0;
}

void configSecondTimer(void (*timerISR)(void *))
{
    timer_config_t t_cfg;
    t_cfg.alarm_en = TIMER_ALARM_EN;
    t_cfg.auto_reload = TIMER_AUTORELOAD_EN;
    t_cfg.counter_dir = TIMER_COUNT_UP;
    t_cfg.counter_en = TIMER_PAUSE;
    t_cfg.divider = 80; // Prescaler → 80 MHz / 80 = 1 MHz → 1 tick = 1 µs
    t_cfg.intr_type = TIMER_INTR_LEVEL;

    timer_init(TIMER_GROUP_0, TIMER_1, &t_cfg);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 1000000ULL);
    timer_set_auto_reload(TIMER_GROUP_0, TIMER_1, TIMER_AUTORELOAD_EN);
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);
    timer_isr_register(TIMER_GROUP_0, TIMER_1, timerISR, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, TIMER_1);
}