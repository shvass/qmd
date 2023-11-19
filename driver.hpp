// rc_car - esp-idf RC driver libraries 
// Copyright (C) 2023  akshay bansod <akshayb@gmx.com>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
#ifndef DRIVER_HPP
#define DRIVER_HPP


#include <driver/mcpwm_prelude.h>


/**
 * @brief motor specific configurations
 */
#define MOTOR_MIN_PULSEWIDTH_US 10      // Minimum pulse width in microseconds 
#define MOTOR_MAX_PULSEWIDTH_US 19000   // maximum pulse width in microseconds 
#define MOTOR_TIMEBASE_RESOLUTION_HZ 1 * 1000 * 1000   // clock frequency in Hertz  ( 1 MHz current) 
#define MOTOR_TIMEBASE_PERIOD        20000      // PWM time period in clock ticks   ( 20 ms current) 




/**
 * @brief driver - two channel pwm generator optimized for driving motors
 * 
 * @details Two channel pwm generator optimized such that it uses single timer
 *  to compare and driver two pwm signal simulataneously.
 */
class driver{

    public:

    /**
     * @brief configures peripherals to generate pwm at mentioned GPIOs.
     * 
     * @param left  GPIO pin for left pwm output
     * @param right  GPIO pin for right pwm output
     */
    driver(int left, int right);


    /**
     * @brief mapping function to map normalized input (0 to 1) to ( MIN - MAX).
     * 
     * @param norm normalized float input strictly between 0.0f to 1.0f 
     * @return int - pwm uptime duration in terms of clock ticks.
     */
    static int map(float norm){
        return (norm) * (MOTOR_MAX_PULSEWIDTH_US - MOTOR_MIN_PULSEWIDTH_US) + MOTOR_MIN_PULSEWIDTH_US;
    }


    /**
     * @brief set left and right motor speeds simulatneously
     * 
     * @param left noramalized left motor speed
     * @param right noramalized right motor speed
     */
    void set(float left = -1, float right = -1);

    /**
     * @brief Set the Left motor speed
     * 
     * @param left noramalized left motor speed
     */
    void setLeft(float left);

    /**
     * @brief Set the Left motor speed
     * 
     * @param left noramalized left motor speed
     */
    void setRight(float right);



    private:

    // mcpwm handlers for internal use  

    /// @brief  pcmwm timers
    mcpwm_timer_handle_t timer;

    /// @brief mcpwm operators
    mcpwm_oper_handle_t lop = nullptr, rop = nullptr;
    /// @brief mcpwm comparators
    mcpwm_cmpr_handle_t lcmp = nullptr, rcmp = nullptr;
    /// @brief mcpwm generators
    mcpwm_gen_handle_t lgen = nullptr, rgen = nullptr;

};


#endif// DRIVER_HPP
