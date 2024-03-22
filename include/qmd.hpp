// qmd - esp-idf quad motor driver 
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
#define MOTOR_MIN_PULSEWIDTH_US 500      // Minimum pulse width in microseconds 
#define MOTOR_MAX_PULSEWIDTH_US 2500   // maximum pulse width in microseconds 
#define MOTOR_TIMEBASE_RESOLUTION_HZ 1 * 1000 * 1000   // clock frequency in Hertz  ( 1 MHz current) 
#define MOTOR_TIMEBASE_PERIOD        20000      // PWM time period in clock ticks   ( 20 ms current) 

#define MOTOR_COUNT_MAX 6  // maximum number of motor parallelly operated by this chipset


/**
 * @brief qmd - quad channel motor driver
 * 
 * @details Quad channel pwm generator optimized such that it uses single mcpwm esp32 peripheral
 *  to compare and driver quad pwm signal simulataneously.
 * 
 * to driver a motor, the module generates one pwm signal and one direction signal.
 */
class qmd{

    public:

    static int def_pwm_pins[], def_dir_pins[];

    /**
     * @brief configures peripherals to generate pwm at mentioned GPIOs.
     * @param count number of channels to operate in parallel (max 4)
     * @param pwmPins  array of GPIO pin numbers for pwm output
     * @param dirPins  array of GPIO pin numbers for direction output
     */
    qmd(int count = 0, int pwmPins[] = def_pwm_pins, int dirPins[] = def_dir_pins);


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
     * @brief speeds of the motors to set must be strictly between  
     * 
     */
    float speeds[MOTOR_COUNT_MAX];

    void update();

    private:

    int dirPins[MOTOR_COUNT_MAX];
    int pwmPins[MOTOR_COUNT_MAX];
    // mcpwm handlers for internal use  

    /// @brief  pcmwm timers
    mcpwm_timer_handle_t timer;

    /// @brief mcpwm operators
    mcpwm_oper_handle_t ops[3] = {nullptr};
    /// @brief mcpwm comparators
    mcpwm_cmpr_handle_t cmps[MOTOR_COUNT_MAX] = {nullptr};
    /// @brief mcpwm generators
    mcpwm_gen_handle_t gens[MOTOR_COUNT_MAX] = {nullptr};

};



#endif// DRIVER_HPP
