//
// Created by rfeldhans on 14.08.18.
//

#include "../include/utils.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#define MAX_DB 96


namespace utils {


    float calculate_db(float* audio_sample, size_t nframes) {
        // find maximum absolute value using OpenMp
        float max_val = 0.0;
        float min_val = 0.0;
        for (int i = 0; i < nframes; i++) {
            if(audio_sample[i] > max_val)
                max_val = audio_sample[i];
            else if (audio_sample[i] < min_val)
                min_val = audio_sample[i];
        }

        if (-1*min_val > max_val){
            max_val = -1*min_val;
        }

        // calculate decibel as found somewhere on stackoverflow
        float dB = MAX_DB + 20 * log10(max_val);

        return dB;
    }

    void read_config(config &config, std::string config_file) {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(config_file, pt);
        config.db_min = pt.get<double>("db_min");
        config.db_keep_alive = pt.get<double>("db_keep_alive");
        config.time_max = ros::Duration(pt.get<double>("time_max")/1000);
        config.time_keep_alive = ros::Duration(pt.get<double>("time_keep_alive")/1000);
        config.esiaf_input_topic = pt.get<std::string>("esiaf_input_topic");
        config.esiaf_output_topic = pt.get<std::string>("esiaf_output_topic");

        config.ros_node_name = pt.get<std::string>("ros_node_name");
        config.ros_publish_db_interval = ros::Duration(pt.get<double>("ros_publish_db_interval")/1000);
        config.ros_decibel_publish_topic = pt.get<std::string>("ros_decibel_publish_topic");
        config.ros_timestamp_publish_topic = pt.get<std::string>("ros_timestamp_publish_topic");
        config.ros_change_config_topic = pt.get<std::string>("ros_change_config_topic");
    }

}