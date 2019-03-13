// esiaf include
#include <esiaf_ros/esiaf_ros.h>

// ros includes
#include "ros/ros.h"
#include "std_msgs/Float32.h"

// inludes from this project
#include "../include/utils.h"
#include "../include/base_segmenter.h"
#include "../include/double_threshold_segmenter.h"

// std includes
#include <mutex>


boost::function<void(const std::vector<int8_t> &, const esiaf_ros::RecordingTimeStamps &)> simple_esiaf_callback;
void esiaf_handler(const std::vector<int8_t> &signal, const esiaf_ros::RecordingTimeStamps & timeStamps){ simple_esiaf_callback(signal, timeStamps); };

utils::config cfg;
segmenter::BaseSegmenter* sgmntr;

float current_dB = 0.0;
std::mutex db_mutex;

int main(int argc, char *argv[]) {
    // parse config
    read_config(cfg, argv[1]);
    sgmntr = new segmenter::DoubleThresholdSegmenter(&cfg);

    // ros initialization
    ros::init(argc, argv, cfg.ros_node_name);
    ros::NodeHandle n;


    //////////////////////////////////////////////////////
    // esiaf start
    //////////////////////////////////////////////////////

    // initialise esiaf
    ROS_INFO("starting esiaf initialisation...");
    esiaf_ros::esiaf_handle *eh = esiaf_ros::initialize_esiaf(&n, esiaf_ros::NodeDesignation::VAD);

    //create format for input topic
    esiaf_ros::EsiafAudioTopicInfo inputTopicInfo;

    esiaf_ros::EsiafAudioFormat allowedFormat;
    allowedFormat.rate = esiaf_ros::Rate::RATE_48000;
    allowedFormat.channels = 1;
    allowedFormat.bitrate = esiaf_ros::Bitrate::BIT_FLOAT_32;
    allowedFormat.endian = esiaf_ros::Endian::LittleEndian;

    inputTopicInfo.allowedFormat = allowedFormat;
    inputTopicInfo.topic = cfg.esiaf_input_topic;

    // create format for output topic
    esiaf_ros::EsiafAudioTopicInfo outputTopicInfo = inputTopicInfo;
    outputTopicInfo.topic = cfg.esiaf_output_topic;

    // notify esiaf about the input topic

    int sampleSizeFactor = sizeof(int8_t) / sizeof(float);

    // here we will segment the audio we acquire from esiaf
    simple_esiaf_callback = [&](const std::vector<int8_t> &signal,
                                const esiaf_ros::RecordingTimeStamps &timeStamps){

        size_t amountFloatFrames = sampleSizeFactor * signal.size();
        float *floatSignal;
        db_mutex.lock();
        current_dB = utils::calculate_db((float*) signal.data(), amountFloatFrames);
        db_mutex.unlock();

        segmenter::BaseSegmenter::SegmentationStatus status;
        sgmntr->segment(floatSignal, amountFloatFrames, status);


        switch (status) {
            case segmenter::BaseSegmenter::SegmentationStatus::finished:
                // add signal for finished segmentation
            case segmenter::BaseSegmenter::SegmentationStatus::started:
                esiaf_ros::publish(eh, outputTopicInfo.topic, signal, timeStamps);
                break;
            case segmenter::BaseSegmenter::SegmentationStatus::idle:
                break;
        }
    };

    // add input topic
    esiaf_ros::add_input_topic(eh, inputTopicInfo, esiaf_handler);

    // notify esiaf about the output topic
    ROS_INFO("adding output topic....");

    // add output topic
    esiaf_ros::add_output_topic(eh, outputTopicInfo);

    // start esiaf
    ROS_INFO("starting esiaf...");

    esiaf_ros::start_esiaf(eh);

    //////////////////////////////////////////////////////
    // esiaf initialisation finished
    //////////////////////////////////////////////////////


    ros::Publisher decibel_pub = n.advertise<std_msgs::Float32>(cfg.ros_decibel_publish_topic, 1, true);
    ROS_INFO("Config:");
    ROS_INFO("db_min = %.2f", cfg.db_min );
    ROS_INFO("db_keep_alive = %.2f", cfg.db_keep_alive);
    ROS_INFO("time_max = %.2f", (double)cfg.time_max.toNSec()/1000000);
    ROS_INFO("time_keep_alive = %.2f" , (double)cfg.time_keep_alive.toNSec()/1000000);


    ROS_INFO("Node ready and rockin'");

    while (ros::ok()) {
        std_msgs::Float32 dB_msg;
        db_mutex.lock();
        dB_msg.data = current_dB;
        db_mutex.unlock();
        decibel_pub.publish(dB_msg);

        cfg.ros_publish_db_interval.sleep();
        ros::spinOnce();
    }

    exit(0);
}