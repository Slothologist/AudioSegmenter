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
        std::string esiaf_input_topic;
        std::string esiaf_output_topic;


        ros::Duration ros_publish_db_interval;
        std::string ros_decibel_publish_topic;
        std::string ros_timestamp_publish_topic;
        std::string ros_change_config_topic;
    };

    /**
     * Calculates the decibel of a specific audio_sample
     * @param audio_sample
     * @return
     */
    float calculate_db(float* audio_sample, size_t nframes);

    void read_config(config& config, std::string config_file);

}

#endif //AUDIO_SEGMENTER_UTILS_H