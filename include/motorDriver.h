/**
 * @file motorDriver.h
 * @brief Definition of the MotorDriver class for controlling DC motors using the ESP32.
 * * This class encapsulates the logic to control the direction and speed
 * of a DC motor via an L6205 H-bridge driver.
 * It uses the ESP32's LEDC peripheral (PWM) for speed control (Duty Cycle)
 * and GPIO pins for direction control.
 */

#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <driver/gpio.h>
#include <driver/ledc.h>
#include "esp_log.h"
#include "esp_err.h"
#include "utils.h" // Assumed to define the 'direction' enumeration

/**
 * @brief Class for controlling DC motors with PWM speed control and direction pins.
 */
class MotorDriver
{
private:
    /**
     * @brief Static counter used to assign a unique ID to each MotorDriver instance.
     */
    static uint8_t id;
    
    /**
     * @brief Unique ID of this motor instance (assigned from 'id').
     */
    uint8_t instance_id;
    
    /**
     * @brief Stores the motor's current rotation direction (FORWARD or BACKWARD).
     * @see direction (defined in utils.h)
     */
    direction current_direction;
    
    /**
     * @brief GPIO pins used for the direction signals.
     */
    gpio_num_t IN1_pin, IN2_pin;
    
    /**
     * @brief The LEDC channel utilized to generate the PWM signal for speed control.
     */
    ledc_channel_t channel;

public:
    /**
     * @brief Constructor, Initializes GPIO pins and configures the LEDC channel (PWM) for speed.
     * @param IN1_pin GPIO pin for the IN1 direction signal.
     * @param IN2_pin GPIO pin for the IN2 direction signal.
     * @param PWM_pin GPIO pin to which the PWM speed signal will be assigned.
     * @param timer The LEDC timer to be used for PWM generation.
     * @param channel The LEDC channel to be used for PWM generation.
     * @param frequency The PWM signal frequency in Hz (e.g., 5000 Hz).
     */
    MotorDriver(gpio_num_t IN1_pin, gpio_num_t IN2_pin, gpio_num_t PWM_pin, 
                ledc_timer_t timer, ledc_channel_t channel, uint16_t frequency);
    
    /**
     * @brief Sets the motor's duty cycle, controlling its speed.
     * @param percentage The duty cycle percentage, from 0,0 to 100,0.
     */
    void setDuty(double percentage);
    
    /**
     * @brief Sets the motor's direction of rotation (FORWARD or BACKWARD).
     * @param dir The new direction to set.
     * @see direction (defined in utils.h)
     */
    void setDirection(direction dir);
    
    /**
     * @brief Toggles the current direction of the motor.
     */
    void toggleDirection();
    
    /**
     * @brief Stops the motor (Does not Change duty Cycle (breaking power)).
     */
    void stop();
};

#endif // MOTORDRIVER_H