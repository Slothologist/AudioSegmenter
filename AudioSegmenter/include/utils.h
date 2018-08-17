//
// Created by rfeldhans on 14.08.18.
//

#ifndef AUDIO_SEGMENTER_UTILS_H
#define AUDIO_SEGMENTER_UTILS_H

#include <jack/jack.h>
#include <string>
#include <boost/chrono/chrono.hpp>

namespace utils{

    struct config{
        double db_min;
        double db_keep_alive;
        boost::chrono::system_clock::duration time_max;
        boost::chrono::system_clock::duration time_keep_alive;
        std::string ros_node_name;
        int ros_publish_db_interval;
        std::string ros_decibel_publish_topic;
        const char* jack_client_name;
        const char* jack_server_name = nullptr;
    };

    /**
     * Calculates the decibel of a specific jack_default_audio_sample_t
     * @param audio_sample
     * @return
     */
    float calculate_db(jack_default_audio_sample_t* audio_sample, jack_nframes_t nframes);

    void read_config(config* config, std::string config_file);

}

#endif //AUDIO_SEGMENTER_UTILS_H