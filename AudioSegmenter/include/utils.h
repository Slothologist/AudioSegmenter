//
// Created by rfeldhans on 14.08.18.
//

#ifndef AUDIO_SEGMENTER_UTILS_H
#define AUDIO_SEGMENTER_UTILS_H

#include <jack/jack.h>
#include <string>
#include "ros/ros.h"

namespace utils{

    struct config{
        double db_min;
        double db_keep_alive;
        ros::Duration time_max;
        ros::Duration time_keep_alive;
        std::string ros_node_name;
        ros::Duration ros_publish_db_interval;
        std::string ros_decibel_publish_topic;
        std::string ros_timestamp_publish_topic;
        std::string ros_change_config_topic;
        std::string jack_client_name;
        std::string jack_server_name;
        std::string jack_input_port_name;
        std::string jack_output_port_name;
    };

    /**
     * Calculates the decibel of a specific jack_default_audio_sample_t
     * @param audio_sample
     * @return
     */
    float calculate_db(jack_default_audio_sample_t* audio_sample, jack_nframes_t nframes);

    void read_config(config& config, std::string config_file);

}

#endif //AUDIO_SEGMENTER_UTILS_H