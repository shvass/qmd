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

#ifndef QUADRATURE_HPP
#define QUADRATURE_HPP


#include <driver/pulse_cnt.h>


#define DECODER_MAX_WHEEL_COUNT 4
#define DECODER_GLITCH_NS (10000)

/**
 * @brief multichannel rotary decoder
 * @todo 
 * instantiate a decoder object specifying all phase A and phase B channels
 * wheel encoder readings will be latched on count array by calling update() frequently
 */
class decoder
{

public:

    /**
     * @brief Construct a new decoder object
     * 
     * @param phaseA array of pins used for phaseA for all encoder channels
     * @param phaseB array of pins used for phaseB for all encoder channels
     * @param count number of parallel channels in use
     */
    decoder(int* phaseA, int* phaseB, int count = DECODER_MAX_WHEEL_COUNT);


    ///@brief readings for each channel, call update @ref update
    float count[DECODER_MAX_WHEEL_COUNT];


    /// @brief update latest readings into count 
    void update();

    ~decoder();

private:

    pcnt_unit_handle_t pcnt_unit[DECODER_MAX_WHEEL_COUNT];
    pcnt_channel_handle_t pcnt_channels[DECODER_MAX_WHEEL_COUNT];

    int unitCount = 0;
};

#endif // QUADRATURE_HPP