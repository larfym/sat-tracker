#ifndef REED_SW_H
#define REED_SW_H

#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include "utils.h"

/** @brief APB Clock Frequency in MHz (used for PCNT configuration). */
#define APB_CLK_FREQ_MHZ 80

/** @brief Debounce filter limit for the Reed Switch (maximum value: 1023). */
#define REED_DEBOUNCE 1023

/**
 * @class ReedSwitch
 * @brief Class for interfacing with a Reed Switch using the ESP32's Pulse Counter (PCNT) peripheral.
 *
 * This class configures a GPIO pin as an input for the Reed Switch,
 * sets up a PCNT unit and channel to count pulses (e.g., for rotation or events),
 * and handles configurations like debouncing and event limits.
 */
class ReedSwitch
{
private:
    gpio_num_t pin;             /**< GPIO pin connected to the Reed Switch. */
    pcnt_unit_t unit;           /**< PCNT unit assigned to this sensor. */
    pcnt_channel_t channel;     /**< PCNT channel within the unit. */
    direction current_dir;      /**< Current counting direction (defined in utils.h). */

public:
    /**
     * @brief Constructor for the ReedSwitch class.
     *
     * @param pin The GPIO pin connected to the Reed Switch.
     * @param unit The PCNT unit to use (e.g., PCNT_UNIT_0).
     * @param channel The PCNT channel to use (e.g., PCNT_CHANNEL_0).
     * @param l_lim The low limit value for generating a PCNT event interrupt.
     * @param h_lim The high limit value for generating a PCNT event interrupt.
     * @param eventHandler A callback function to be executed when a limit event occurs.
     */
    ReedSwitch(gpio_num_t pin, pcnt_unit_t unit, pcnt_channel_t channel, uint16_t l_lim, uint16_t h_lim, void (*eventHandler)(void *));

    /**
     * @brief Gets the current pulse count value.
     * @return The accumulated count value.
     */
    int16_t getCount();

    /**
     * @brief Stops the pulse counter.
     */
    void stopCount();

    /**
     * @brief Starts the pulse counter.
     */
    void startCount();

    /**
     * @brief Resets the pulse counter to zero.
     */
    void resetCount();

    /**
     * @brief Sets the counting direction for the PCNT unit.
     * @param dir The new counting direction (e.g., UP, DOWN, or ZERO).
     */
    void countDirection(direction dir);

    /**
     * @brief Gets the PCNT unit assigned to this sensor.
     * @return The PCNT unit ID.
     */
    pcnt_unit_t getUnit();
};

#endif // REED_SW_H