/**
 * Jackaudio part taken mostly from https://github.com/jackaudio/jack2/blob/master/example-clients/thru_client.c
 */

#include <cstdio>
#include <cstdlib>
#include <jack/jack.h>
#include <boost/thread.hpp>

// ros includes
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "speech_rec_pipeline_msgs/SegmenterConfig.h"
#include "speech_rec_pipeline_msgs/SegmentedAudioTimeStamps.h"

// inludes from this project
#include "../include/utils.h"
#include "../include/base_segmenter.h"
#include "../include/double_threshold_segmenter.h"


jack_port_t *input_port;
jack_port_t *output_port;
jack_client_t *client;

utils::config cfg;
segmenter::BaseSegmenter* sgmntr;

ros::Publisher timeframe_pub;
float current_dB = 0.0;


/**
 * The process callback for this JACK application is called in a
 * special realtime thread once for each audio cycle.
 *
 * This client follows a simple rule: when the JACK transport is
 * running, copy the input port to the output.  When it stops, exit.
 */


void publish_segmented_timesteps(ros::Time start, ros::Publisher* publisher) {
    ROS_INFO("Segmentation took place!");
    speech_rec_pipeline_msgs::SegmentedAudioTimeStamps msg;
    msg.start = start;
    msg.finish = ros::Time::now();
    publisher->publish(msg);
}

int process(jack_nframes_t nframes, void *arg) {
    jack_default_audio_sample_t *in, *out;
    in = (jack_default_audio_sample_t *) jack_port_get_buffer(input_port, nframes);
    out = (jack_default_audio_sample_t *) jack_port_get_buffer(output_port, nframes);
    current_dB = utils::calculate_db(in, nframes);

    segmenter::BaseSegmenter::SegmentationStatus status;
    sgmntr->segment(in, nframes, status);

    if (status == segmenter::BaseSegmenter::SegmentationStatus::started) {
        memcpy(out, in, nframes * sizeof(jack_default_audio_sample_t));
        return 0;
    } else if (status == segmenter::BaseSegmenter::SegmentationStatus::finished){
        boost::thread publisher_thread(publish_segmented_timesteps, sgmntr->get_last_started(), &timeframe_pub);
    }
    // set out to zero
    float* nullarray = new float[nframes]();
    memcpy(out, nullarray, nframes * sizeof(jack_default_audio_sample_t));

    return 0;
}

/**
 * JACK calls this shutdown_callback if the server ever shuts down or
 * decides to disconnect the client.
 */
void jack_shutdown(void *arg) {
    exit(1);
}

bool change_config(speech_rec_pipeline_msgs::SegmenterConfig::Request &req,
                   speech_rec_pipeline_msgs::SegmenterConfig::Response &res) {
    cfg.db_min = req.db_min;
    cfg.db_keep_alive = req.db_keep_alive;
    cfg.time_max = ros::Duration(((double)req.time_max)/1000);
    cfg.time_keep_alive = ros::Duration(((double)req.time_keep_alive)/1000);
    return true;
}

int main(int argc, char *argv[]) {
    // parse config
    read_config(cfg, argv[1]);
    sgmntr = new segmenter::DoubleThresholdSegmenter(&cfg);

    // ros stuff
    ros::init(argc, argv, cfg.ros_node_name);
    ros::NodeHandle n;
    ros::Publisher decibel_pub = n.advertise<std_msgs::Float32>(cfg.ros_decibel_publish_topic, 1, true);
    timeframe_pub = n.advertise<speech_rec_pipeline_msgs::SegmentedAudioTimeStamps>(cfg.ros_timestamp_publish_topic, 1, true);
    ros::ServiceServer change_config_service = n.advertiseService(cfg.ros_change_config_topic, change_config);
    ROS_INFO("Config:");
    ROS_INFO("db_min = %.2f", cfg.db_min );
    ROS_INFO("db_keep_alive = %.2f", cfg.db_keep_alive);
    ROS_INFO("time_max = %.2f", (double)cfg.time_max.toNSec()/1000000);
    ROS_INFO("time_keep_alive = %.2f" , (double)cfg.time_keep_alive.toNSec()/1000000);

    // Jack stuff
    auto jack_server_name = (int) JackNullOption;
    const char **ports;
    const char *client_name = cfg.jack_client_name.c_str();
    const char *server_name = cfg.jack_server_name == "default" ? nullptr : cfg.jack_server_name.c_str();
    jack_options_t options = JackNullOption;
    jack_status_t status;

    if (server_name != nullptr) {
        jack_server_name |= JackServerName;
        options = (jack_options_t) jack_server_name;
    }

    /* open a client connection to the JACK server */
    client = jack_client_open(client_name, options, &status, server_name);
    if (client == nullptr) {
        ROS_ERROR("jack_client_open() failed, "
                        "status = 0x%2.0x", status);
        if (status & JackServerFailed) {
            ROS_ERROR("Unable to connect to JACK server");
        }
        exit(1);
    }
    if (status & JackServerStarted) {
        ROS_INFO("JACK server started");
    }
    if (status & JackNameNotUnique) {
        client_name = jack_get_client_name(client);
        ROS_ERROR("unique name `%s' assigned", client_name);
    }

    /* tell the JACK server to call `process()' whenever
       there is work to be done.
    */

    jack_set_process_callback(client, process, nullptr);

    /* tell the JACK server to call `jack_shutdown()' if
       it ever shuts down, either entirely, or if it
       just decides to stop calling us.
    */

    jack_on_shutdown(client, jack_shutdown, nullptr);
    input_port = jack_port_register(client, cfg.jack_input_port_name.c_str(), JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
    output_port = jack_port_register(client, cfg.jack_output_port_name.c_str(), JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
    if ((input_port == nullptr) || (output_port == nullptr)) {
        ROS_ERROR("no more JACK ports available");
        exit(1);
    }

    /* Tell the JACK server that we are ready to roll.  Our
     * process() callback will start running now. */

    if (jack_activate(client)) {
        ROS_ERROR("cannot activate client");
        exit(1);
    }

    /* Connect the ports.  You can't do this before the client is
     * activated, because we can't make connections to clients
     * that aren't running.  Note the confusing (but necessary)
     * orientation of the driver backend ports: playback ports are
     * "input" to the backend, and capture ports are "output" from
     * it.
     */

    ports = jack_get_ports(client, nullptr, nullptr, JackPortIsPhysical | JackPortIsOutput);
    if (ports == nullptr) {
        ROS_ERROR("no physical capture ports");
        exit(1);
    }

    if (jack_connect(client, ports[0], jack_port_name(input_port)))
        ROS_ERROR("cannot connect input ports");

    free(ports);

    ports = jack_get_ports(client, nullptr, nullptr, JackPortIsPhysical | JackPortIsInput);
    if (ports == nullptr) {
        ROS_ERROR( "no physical playback ports");
        exit(1);
    }

    if (jack_connect(client, jack_port_name(output_port), ports[0]))
        ROS_ERROR( "cannot connect input ports");

    free(ports);


    /* keep running until the transport stops */
    ROS_INFO("Node ready and rockin'");

    while (ros::ok()) {
        std_msgs::Float32 dB_msg;
        dB_msg.data = current_dB;
        decibel_pub.publish(dB_msg);

        cfg.ros_publish_db_interval.sleep();
    }

    jack_client_close(client);
    exit(0);
}