/**
 * Taken mostly from https://github.com/jackaudio/jack2/blob/master/example-clients/thru_client.c
 */

#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <jack/jack.h>
#include <chrono>
#include <thread>
#include <boost/chrono/chrono.hpp>
#include <iostream>

#include "../include/utils.h"

#define NOW boost::chrono::system_clock::now()

jack_port_t **input_ports;
jack_port_t **output_ports;
jack_client_t *client;

utils::config cfg;

boost::chrono::system_clock::time_point last_start;
boost::chrono::system_clock::time_point last_keep_alive;
bool started_segmentation = false;
double current_dB = 0.0;

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

int process(jack_nframes_t nframes, void *arg) {
    int i;
    jack_default_audio_sample_t *in, *out;
    for (i = 0; i < 2; i++) {
        in = (jack_default_audio_sample_t *) jack_port_get_buffer(input_ports[i], nframes);
        out = (jack_default_audio_sample_t *) jack_port_get_buffer(output_ports[i], nframes);

        current_dB = utils::calculate_db(in, nframes);
        if (started_segmentation) {
            if (NOW - last_start > cfg.time_max) {
                // max time reached
                started_segmentation = false;
                continue;

            } else if (current_dB < cfg.db_keep_alive) {
                if (NOW - last_keep_alive > cfg.time_keep_alive) {
                    // signal not loud enough anymore
                    started_segmentation = false;
                    continue;

                } else {
                    // signal still loud enough
                    last_keep_alive = NOW;
                }
            } else {
                // signal comfortably loud enough
                last_keep_alive = NOW;

            }

        } else if (current_dB >= cfg.db_min) {
            // segmentation starting
            last_start = NOW;
            last_keep_alive = last_start;
            started_segmentation = true;

        } else {
            continue;
        }

        memcpy(out, in, nframes * sizeof(jack_default_audio_sample_t));
    }
    return 0;
}

/**
 * JACK calls this shutdown_callback if the server ever shuts down or
 * decides to disconnect the client.
 */
void jack_shutdown(void *arg) {
    free(input_ports);
    free(output_ports);
    exit(1);
}

int main(int argc, char *argv[]) {
    // parse config
    read_config(&cfg, argv[1]);


    int i;
    auto jack_server_name = (int) JackNullOption;
    const char **ports;
    const char *client_name;
    const char *server_name = nullptr;
    jack_options_t options = JackNullOption;
    jack_status_t status;

    if (argc >= 3)        /* client name specified? */
    {
        client_name = argv[2];
        if (argc >= 4)    /* server name specified? */
        {
            server_name = argv[3];
            jack_server_name |= JackServerName;
            options = (jack_options_t) jack_server_name;
        }
    } else              /* use basename of argv[0] */
    {
        client_name = strrchr(argv[0], '/');
        if (client_name == nullptr) {
            client_name = argv[0];
        } else {
            client_name++;
        }
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

    /* create two ports pairs*/
    input_ports = (jack_port_t **) calloc(2, sizeof(jack_port_t *));
    output_ports = (jack_port_t **) calloc(2, sizeof(jack_port_t *));

    char port_name[16];
    for (i = 0; i < 2; i++) {
        sprintf(port_name, "input_%d", i + 1);
        input_ports[i] = jack_port_register(client, port_name, JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
        sprintf(port_name, "output_%d", i + 1);
        output_ports[i] = jack_port_register(client, port_name, JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
        if ((input_ports[i] == nullptr) || (output_ports[i] == nullptr)) {
            fprintf(stderr, "no more JACK ports available\n");
            exit(1);
        }
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

    for (i = 0; i < 2; i++)
        if (jack_connect(client, ports[i], jack_port_name(input_ports[i])))
            fprintf(stderr, "cannot connect input ports\n");

    free(ports);

    ports = jack_get_ports(client, nullptr, nullptr, JackPortIsPhysical | JackPortIsInput);
    if (ports == nullptr) {
        fprintf(stderr, "no physical playback ports\n");
        exit(1);
    }

    for (i = 0; i < 2; i++)
        if (jack_connect(client, jack_port_name(output_ports[i]), ports[i]))
            fprintf(stderr, "cannot connect input ports\n");

    free(ports);

    /* install a signal handler to properly quits jack client */
    signal(SIGQUIT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGHUP, signal_handler);
    signal(SIGINT, signal_handler);

    /* keep running until the transport stops */

    while (true) {
        std::cout << current_dB << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg.publish_db_every_ms));
    }

    jack_client_close(client);
    exit(0);
}