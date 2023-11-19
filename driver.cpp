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
#include "driver.hpp"

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


driver::driver(int left, int right) {

    // common timer for both pwm drivers
    mcpwm_new_timer(&timer_config, &timer);

    mcpwm_new_operator(&operator_config, &lop);
    mcpwm_new_operator(&operator_config, &rop);

    mcpwm_operator_connect_timer(lop, timer);
    mcpwm_operator_connect_timer(rop, timer);


    mcpwm_new_comparator(lop, &comparator_config, &lcmp);
    mcpwm_new_comparator(rop, &comparator_config, &rcmp);

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = left,
        .flags = {}
    };

    mcpwm_new_generator(lop, &generator_config, &lgen);

    generator_config.gen_gpio_num = right;
    mcpwm_new_generator(rop, &generator_config, &rgen);



    // left comparator operations
    // go high on counter empty
    mcpwm_generator_set_action_on_timer_event(lgen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    
    // go low on compare threshold
    mcpwm_generator_set_action_on_compare_event(lgen,
              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, lcmp, MCPWM_GEN_ACTION_LOW));


    // right comparator operations
    // go high on counter empty
    mcpwm_generator_set_action_on_timer_event(rgen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    // go low on compare threshold
    mcpwm_generator_set_action_on_compare_event(rgen,
              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, rcmp, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
};


void driver::set(float left, float right){
    if(left  >  0.0f && left  <= 1.0f) mcpwm_comparator_set_compare_value(lcmp, map(left));
    if(right > 0.0f && right <= 1.0f) mcpwm_comparator_set_compare_value(rcmp, map(right));
};


void driver::setLeft(float left){
    mcpwm_comparator_set_compare_value(lcmp, map(left));
}


void driver::setRight(float right){
    mcpwm_comparator_set_compare_value(rcmp, map(right));
};