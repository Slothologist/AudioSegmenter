/**
 * Jackaudio part taken mostly from https://github.com/jackaudio/jack2/blob/master/example-clients/thru_client.c
 */

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <jack/jack.h>
#include <boost/thread.hpp>

// ros includes
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "speech_rec_pipeline_msgs/SegmenterConfig.h"
#include "speech_rec_pipeline_msgs/SegmentedAudioTimeStamps.h"

// inludes from this project
#include "../include/utils.h"


jack_port_t *input_port;
jack_port_t *output_port;
jack_client_t *client;

utils::config* cfg;

ros::Time last_start;
ros::Time last_keep_alive;
ros::Publisher timeframe_pub;
bool started_segmentation = false;
float current_dB = 0.0;

static void signal_handler(int sig) {
    jack_client_close(client);
    fprintf(stderr, "signal received, exiting ...\n");
    exit(0);
}

/**
 * The process callback for this JACK application is called in a
 * special realtime thread once for each audio cycle.
 *
 * This client follows a simple rule: when the JACK transport is
 * running, copy the input port to the output.  When it stops, exit.
 */


void publish_segmented_timesteps(ros::Time start, ros::Time finish, ros::Publisher* publisher) {
    ROS_INFO("Segmentation took place!");
    speech_rec_pipeline_msgs::SegmentedAudioTimeStamps msg;
    msg.start = start;
    msg.finish = finish;
    publisher->publish(msg);
}

int process(jack_nframes_t nframes, void *arg) {
    bool segmentation_finished = false;
    jack_default_audio_sample_t *in, *out;
    in = (jack_default_audio_sample_t *) jack_port_get_buffer(input_port, nframes);
    out = (jack_default_audio_sample_t *) jack_port_get_buffer(output_port, nframes);
    ros::Time now = ros::Time::now();

    current_dB = utils::calculate_db(in, nframes);
    if (started_segmentation) {
        if (current_dB > cfg->db_keep_alive) {
            if (now - last_start > cfg->time_max) {
                // max time reached
                started_segmentation = false;
                segmentation_finished = true;
            } else {
                last_keep_alive = now;
            }
        } else {
            if (now - last_start > cfg->time_max) {
                // max time reached
                started_segmentation = false;
                segmentation_finished = true;
            } else if (now - last_keep_alive > cfg->time_keep_alive) {
                // signal not loud enough anymore
                started_segmentation = false;
                segmentation_finished = true;
            }
        }
    } else if(current_dB > cfg->db_min){
        started_segmentation = true;
        last_keep_alive = now;
        last_start = now;
    }


    if (started_segmentation) {
        memcpy(out, in, nframes * sizeof(jack_default_audio_sample_t));
    } else if (segmentation_finished){
        boost::thread publisher_thread(publish_segmented_timesteps, last_start, now, &timeframe_pub);
    }

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
    cfg->db_min = req.db_min;
    cfg->db_keep_alive = req.db_keep_alive;
    cfg->time_max = ros::Duration(((double)req.time_max)/1000);
    cfg->time_keep_alive = ros::Duration(((double)req.time_keep_alive)/1000);
    return true;
}

int main(int argc, char *argv[]) {
    // parse config
    cfg = new utils::config();
    read_config(cfg, argv[1]);

    // ros stuff
    ros::init(argc, argv, cfg->ros_node_name);
    ros::NodeHandle n;
    ros::Publisher decibel_pub = n.advertise<std_msgs::Float32>(cfg->ros_decibel_publish_topic, 1, true);
    timeframe_pub = n.advertise<speech_rec_pipeline_msgs::SegmentedAudioTimeStamps>(cfg->ros_timestamp_publish_topic, 1, true);
    ros::ServiceServer change_config_service = n.advertiseService(cfg->ros_change_config_topic, change_config);
    ROS_INFO("Config:");
    ROS_INFO("db_min = %.2f\n"
             "db_keep_alive = %.2f\n"
             "time_max = %.2f\n"
             "time_keep_alive = %.2f", cfg->db_min, cfg->db_keep_alive, (double)cfg->time_max.toNSec()/1000000, (double)cfg->time_keep_alive.toNSec()/1000000);

    // Jack stuff
    auto jack_server_name = (int) JackNullOption;
    const char **ports;
    const char *client_name = cfg->jack_client_name;
    const char *server_name = cfg->jack_server_name;
    jack_options_t options = JackNullOption;
    jack_status_t status;

    if (server_name != nullptr) {
        jack_server_name |= JackServerName;
        options = (jack_options_t) jack_server_name;
    }

    /* open a client connection to the JACK server */
    client = jack_client_open(client_name, options, &status, server_name);
    if (client == nullptr) {
        fprintf(stderr, "jack_client_open() failed, "
                        "status = 0x%2.0x\n", status);
        if (status & JackServerFailed) {
            fprintf(stderr, "Unable to connect to JACK server\n");
        }
        exit(1);
    }
    if (status & JackServerStarted) {
        fprintf(stderr, "JACK server started\n");
    }
    if (status & JackNameNotUnique) {
        client_name = jack_get_client_name(client);
        fprintf(stderr, "unique name `%s' assigned\n", client_name);
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

    char port_name[16];
    sprintf(port_name, "input_%d", 1);
    input_port = jack_port_register(client, port_name, JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
    sprintf(port_name, "output_%d", 1);
    output_port = jack_port_register(client, port_name, JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
    if ((input_port == nullptr) || (output_port == nullptr)) {
        fprintf(stderr, "no more JACK ports available\n");
        exit(1);
    }

    /* Tell the JACK server that we are ready to roll.  Our
     * process() callback will start running now. */

    if (jack_activate(client)) {
        fprintf(stderr, "cannot activate client");
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
        fprintf(stderr, "no physical capture ports\n");
        exit(1);
    }

    if (jack_connect(client, ports[0], jack_port_name(input_port)))
        fprintf(stderr, "cannot connect input ports\n");

    free(ports);

    ports = jack_get_ports(client, nullptr, nullptr, JackPortIsPhysical | JackPortIsInput);
    if (ports == nullptr) {
        fprintf(stderr, "no physical playback ports\n");
        exit(1);
    }

    if (jack_connect(client, jack_port_name(output_port), ports[0]))
        fprintf(stderr, "cannot connect input ports\n");

    free(ports);

    /* install a signal handler to properly quits jack client */
    signal(SIGQUIT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGHUP, signal_handler);
    signal(SIGINT, signal_handler);

    /* keep running until the transport stops */
    ROS_INFO("Node ready and rockin'");

    while (ros::ok()) {
        std_msgs::Float32 dB_msg;
        dB_msg.data = current_dB;
        decibel_pub.publish(dB_msg);

        cfg->ros_publish_db_interval.sleep();
    }

    jack_client_close(client);
    exit(0);
}