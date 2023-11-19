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
#include "qmd.hpp"
#include "driver/gpio.h"

// convert pin number to bit mask
#define GET_BIT(PIN)  (1ULL<<PIN) 

/// @brief timer configuration
mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = MOTOR_TIMEBASE_RESOLUTION_HZ,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks = MOTOR_TIMEBASE_PERIOD,
    .flags = {}
};

/// @brief operator config
mcpwm_operator_config_t operator_config = {
    .group_id = 0, // operator must be in the same group to the timer
    .flags = {}
};


mcpwm_comparator_config_t comparator_config = {
    .flags = {1, 0, 0}
};


qmd::qmd(int count, int pwmPins[], int dirPins[]) {

    
    for(int i = 0; i < 4; i++) this->dirPins[i] = dirPins[i];

    // common timer for both pwm drivers
    mcpwm_new_timer(&timer_config, &timer);

    mcpwm_new_operator(&operator_config, &ops[0]);
    mcpwm_new_operator(&operator_config, &ops[1]);

    mcpwm_operator_connect_timer(ops[0], timer);
    mcpwm_operator_connect_timer(ops[1], timer);


    mcpwm_new_comparator(ops[0], &comparator_config, &cmps[0]);
    mcpwm_new_comparator(ops[0], &comparator_config, &cmps[1]);
    mcpwm_new_comparator(ops[1], &comparator_config, &cmps[2]);
    mcpwm_new_comparator(ops[1], &comparator_config, &cmps[3]);

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = pwmPins[0],
        .flags = {}
    };
    
    generator_config.gen_gpio_num = pwmPins[0];
    mcpwm_new_generator(ops[0], &generator_config, &gens[0]);
    generator_config.gen_gpio_num = pwmPins[1];
    mcpwm_new_generator(ops[0], &generator_config, &gens[1]);
    generator_config.gen_gpio_num = pwmPins[2];
    mcpwm_new_generator(ops[1], &generator_config, &gens[2]);
    generator_config.gen_gpio_num = pwmPins[3];
    mcpwm_new_generator(ops[1], &generator_config, &gens[3]);




    // left comparator operations
    // go high on counter empty

    for(int i = 0; i < 4; i++){

        mcpwm_generator_set_action_on_timer_event(gens[i],
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    
        // go low on compare threshold
        mcpwm_generator_set_action_on_compare_event(gens[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmps[i], MCPWM_GEN_ACTION_LOW));


    }

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = GET_BIT(dirPins[0]) | GET_BIT(dirPins[1]) | GET_BIT(dirPins[2]) | GET_BIT(dirPins[3]),
        .mode = GPIO_MODE_OUTPUT,               
        .pull_up_en = GPIO_PULLUP_DISABLE,      
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE 
    };

    gpio_config(&gpio_cfg);
    
};



void qmd::update(){

    float speed = 0.0f;

    for( int  i = 0; i < 4; i++) {
        speed = speeds[i];
        if(speed < 0.0f) {
            gpio_set_level((gpio_num_t) dirPins[i], 1);
            speed += 1.0f;
        }
        else gpio_set_level((gpio_num_t) dirPins[i], 0);
        
        mcpwm_comparator_set_compare_value(cmps[i], map(speed));
    };
};