//
// Created by rfeldhans on 14.08.18.
//

#ifndef AUDIO_SEGMENTER_UTILS_H
#define AUDIO_SEGMENTER_UTILS_H

#include <jack/jack.h>
#include <string>


struct config{
    double db_min;
    double db_keep_alive;
    int time_max;
    int time_keep_alive;
};

/**
 * Calculates the decibel of a specific jack_default_audio_sample_t
 * @param audio_sample
 * @return
 */
double calculate_db(jack_default_audio_sample_t* audio_sample);

void read_config(config* config, std::string config_file);


#endif //AUDIO_SEGMENTER_UTILS_H