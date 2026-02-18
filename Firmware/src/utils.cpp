#include "utils.h"

static const char *TAG = "UTILS";
static const float A_CONST = 3.0f * PI_FLOAT / 2.0f - atan2f(C_ANT, D_ANT) - atan2f(F_ANT, E_ANT);
static const float B_CONST = 2.0f * sqrtf((F_ANT * F_ANT + E_ANT * E_ANT) * (D_ANT * D_ANT + C_ANT * C_ANT));
static const float C_CONST = F_ANT * F_ANT + E_ANT * E_ANT + D_ANT * D_ANT + C_ANT * C_ANT - A_ANT * A_ANT;
static float elevation_lut_mm[LUT_SIZE + 1], elevation_lut_deg[LUT_SIZE + 1];

bool configSatellite(Sgp4 *satellite)
{
    config.begin("config", false);
    String name = config.getString("name", "");
    String line1 = config.getString("line1", "");
    String line2 = config.getString("line2", "");

    config.end();

    ESP_LOGI(TAG, "Loaded TLE Data -> %s: tle1: %s, tle2: %s", name.c_str(), line1.c_str(), line2.c_str());
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

esfericalAngles_t getSavedOffsets(void)
{
    esfericalAngles_t offsets;
    config.begin("config", false);
    offsets.azimuth = config.getDouble("offset_az", 0.0);
    offsets.elevation = config.getDouble("offset_el", 0.0);
    config.end();
    ESP_LOGI(TAG, "Updated Offsets - Az: %.3f [°], El: %.3f [°]", offsets.azimuth, offsets.elevation);
    return offsets;
}

void saveTLE(String name, String line1, String line2)
{
    config.begin("config", false);
    config.putString("name", name);
    config.putString("line1", line1);
    config.putString("line2", line2);
    config.end();
    ESP_LOGI(TAG, "Data Saved - Name: %s tle1: %s, tle2: %s", name.c_str(), line1.c_str(), line2.c_str());
}

void saveOffsets(double offset_az, double offset_el)
{
    config.begin("config", false);
    config.putDouble("offset_az", offset_az);
    config.putDouble("offset_el", offset_el);
    config.end();
    ESP_LOGI(TAG, "Offsets Saved - offset_az: %f, offset_el: %f\n", offset_az, offset_el);
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

void configControlTimer(void (*timerISR)(void *), uint64_t sample_time_us)
{
    timer_config_t t_cfg;
    t_cfg.alarm_en = TIMER_ALARM_EN;
    t_cfg.auto_reload = TIMER_AUTORELOAD_EN;
    t_cfg.counter_dir = TIMER_COUNT_UP;
    t_cfg.counter_en = TIMER_PAUSE;
    t_cfg.divider = TIMER_PREESCALER; // Prescaler → 80 MHz / 80 = 1 MHz → 1 tick = 1 µs
    t_cfg.intr_type = TIMER_INTR_LEVEL;

    timer_init(TIMER_GROUP_0, TIMER_1, &t_cfg);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, sample_time_us);
    timer_set_auto_reload(TIMER_GROUP_0, TIMER_1, TIMER_AUTORELOAD_EN);
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);
    timer_isr_register(TIMER_GROUP_0, TIMER_1, timerISR, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, TIMER_1);
}

void init_elevation_lut()
{
    for (uint16_t i = 0; i <= LUT_SIZE; i++)
    {
        elevation_lut_mm[i] = elevation_mm_from_deg(i * ANGLE_STEP);
        elevation_lut_deg[i] = elevation_deg_from_mm(i * MM_STEP);
    }
}

float elevation_mm_from_deg_lut(float deg)
{
    if (deg <= 0.0f)
        return elevation_lut_mm[0];
    if (deg >= 90.0f)
        return elevation_lut_mm[LUT_SIZE];

    float pos = deg * INV_DEG_TO_LUT_POS;

    uint16_t idx = (uint16_t)pos;
    float frac = pos - (float)idx;

    // y = y0 + frac * (y1 - y0)
    float y0 = elevation_lut_mm[idx];
    float y1 = elevation_lut_mm[idx + 1];
    return y0 + frac * (y1 - y0);
}

float elevation_deg_from_mm_lut(float mm)
{
    if (mm <= 0.0f)
        return elevation_lut_deg[0];
    if (mm >= MAX_EXTENSION_mm)
        return elevation_lut_deg[LUT_SIZE];

    float pos = mm * INV_MM_TO_LUT_POS;

    uint16_t idx = (uint16_t)pos;
    float frac = pos - (float)idx;

    // y = y0 + frac * (y1 - y0)
    float y0 = elevation_lut_mm[idx];
    float y1 = elevation_lut_mm[idx + 1];
    return y0 + frac * (y1 - y0);
}

float elevation_mm_from_deg(float deg)
{
    return sqrtf(C_CONST - B_CONST * cosf(A_CONST - DEG_TO_RAD * deg)) - BP_ANT;
}

float elevation_deg_from_mm(float mm)
{
    return RAD_TO_DEG * (A_CONST - acosf( (C_CONST - (BP_ANT + mm) * (BP_ANT + mm)) / B_CONST));
}