#include "quadrature.hpp"

#include <esp_log.h>
#include <esp_intr_types.h>

#define LOG "DECODER"

#define PCNT_COUNT_LIMIT_HIGH 1000
#define PCNT_COUNT_LIMIT_LOW -1000


// configuration to initialize all pcnt units
pcnt_unit_config_t pcnt_config = {
    .low_limit = PCNT_COUNT_LIMIT_LOW,
    .high_limit = PCNT_COUNT_LIMIT_HIGH,
    .intr_priority = 0,
    .flags = {true}
};


// configuration to initialize all pcnt channels
pcnt_chan_config_t chan_config = {
    .edge_gpio_num = -1,
    .level_gpio_num = -1,
    .flags = {0, 0, 0, 0, 0}};


pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = DECODER_GLITCH_NS,
};




decoder::decoder(int *phaseA, int *phaseB, int count){

    if(count > DECODER_MAX_WHEEL_COUNT) 
        ESP_LOGD(LOG, "ERROR: unit count exceeded %d", count = DECODER_MAX_WHEEL_COUNT);

    unitCount = count;

    for(int i = 0; i <count; i++){
        pcnt_new_unit(&pcnt_config, &pcnt_unit[i]);
        pcnt_unit_handle_t& current = pcnt_unit[i];


        chan_config.edge_gpio_num = phaseA[i];
        chan_config.level_gpio_num = phaseB[i];

        pcnt_new_channel(current, &chan_config, &pcnt_channels[i]);
        pcnt_channel_set_edge_action(pcnt_channels[i], PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
        pcnt_channel_set_level_action(pcnt_channels[i], PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        
        pcnt_unit_add_watch_point(current, PCNT_COUNT_LIMIT_HIGH);
        pcnt_unit_add_watch_point(current, PCNT_COUNT_LIMIT_LOW);

        pcnt_unit_set_glitch_filter(current, &filter_config);

        pcnt_unit_enable(current);
        pcnt_unit_start(current);
    };

}

void decoder::update(){
    int temp = 0;
    for(int i = 0; i < unitCount; i++){
        pcnt_unit_get_count(pcnt_unit[i], &temp);
        count[i] = temp;
    };
}


void decoder::reset(int index){
    // for any negative index reset all
    if(index < 0){
        for(int i = 0; i < unitCount; i++){
            pcnt_unit_clear_count(pcnt_unit[i]);
            count[i] = 0;
        };
    }

    // check for valid index
    else if(index < unitCount) {
        pcnt_unit_clear_count(pcnt_unit[index]);
        count[index] = 0;
    }
}



decoder::~decoder(){

    for(int i = 0; i < unitCount; i++){
        pcnt_unit_disable(pcnt_unit[i]);
        pcnt_del_channel(pcnt_channels[i]);
        pcnt_del_unit(pcnt_unit[i]);
    }
}
