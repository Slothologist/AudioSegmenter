//
// Created by rfeldhans on 14.08.18.
//

#include "../include/utils.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>



double calculate_db(jack_default_audio_sample_t* audio_sample){
    return 0.0;
}

void read_config(config* config, std::string config_file){

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);
    config->db_min = (double) pt.get<int>("Default.db_min");
    config->db_keep_alive = (double) pt.get<int>("Default.db_keep_alive");
    config->time_max = pt.get<int>("Default.time_max");
    config->time_keep_alive = pt.get<int>("Default.time_keep_alive");
}

