//
// Created by rfeldhans on 14.08.18.
//

#include "../include/utils.h"
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <math.h>

#define MAX_DB 96


namespace utils {


    double calculate_db(jack_default_audio_sample_t *audio_sample, jack_nframes_t nframes) {
        double max = 0.0;
        for (int i = 0; i < nframes; i++) {
            if(audio_sample[i * sizeof(jack_default_audio_sample_t)] > max)
                max = audio_sample[i * sizeof(jack_default_audio_sample_t)];
            else if ( -1 * audio_sample[i * sizeof(jack_default_audio_sample_t)] > max)
                max = -1 * audio_sample[i * sizeof(jack_default_audio_sample_t)];
        }
        double dB = MAX_DB + 20 * log10(max);
        //std::cout << " max: " << max << ", dB: " << dB << std::endl;

        return dB;
    }

    void read_config(config *config, std::string config_file) {

        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(config_file, pt);
        config->db_min = pt.get<double>("db_min");
        config->db_keep_alive = pt.get<double>("db_keep_alive");
        config->time_max = pt.get<int>("time_max");
        config->time_keep_alive = pt.get<int>("time_keep_alive");
    }

}