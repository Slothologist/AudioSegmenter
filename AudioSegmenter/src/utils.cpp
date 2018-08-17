//
// Created by rfeldhans on 14.08.18.
//

#include "../include/utils.h"
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <math.h>

#define MAX_DB 96


namespace utils {


    float calculate_db(jack_default_audio_sample_t *audio_sample, jack_nframes_t nframes) {
        float max = 0.0;
        for (int i = 0; i < nframes; i++) {
            if(audio_sample[i * sizeof(jack_default_audio_sample_t)] > max)
                max = audio_sample[i * sizeof(jack_default_audio_sample_t)];
            else if ( -1 * audio_sample[i * sizeof(jack_default_audio_sample_t)] > max)
                max = -1 * audio_sample[i * sizeof(jack_default_audio_sample_t)];
        }
        float dB = MAX_DB + 20 * log10(max);

        return dB;
    }

    void read_config(config *config, std::string config_file) {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(config_file, pt);
        config->db_min = pt.get<double>("db_min");
        config->db_keep_alive = pt.get<double>("db_keep_alive");
        config->time_max = boost::chrono::milliseconds(pt.get<int>("time_max"));
        config->time_keep_alive = boost::chrono::milliseconds(pt.get<int>("time_keep_alive"));
        config->ros_node_name = pt.get<std::string>("ros_node_name");
        config->ros_publish_db_interval = pt.get<int>("ros_publish_db_interval");
        config->ros_decibel_publish_topic = pt.get<std::string>("ros_decibel_publish_topic");
        config->jack_client_name = pt.get<std::string>("jack_client_name").c_str();
        //config->jack_server_name = pt.get<std::string>("jack_server_name").c_str(); TODO: read if there, don't if not
    }

}