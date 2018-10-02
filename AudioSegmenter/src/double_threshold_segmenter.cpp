//
// Created by rfeldhans on 02.10.18.
//

#include "../include/double_threshold_segmenter.h"

namespace segmenter{

    DoubleThresholdSegmenter::DoubleThresholdSegmenter(utils::config* cfg):
            BaseSegmenter::BaseSegmenter(cfg)
    {

        started_segmentation = false;
    }

    void DoubleThresholdSegmenter::segment(jack_default_audio_sample_t* sample,
                                           jack_nframes_t frame_count,
                                           SegmentationStatus& status) {
        ros::Time now = ros::Time::now();

        float current_dB = utils::calculate_db(sample, frame_count);
        if (started_segmentation) {
            if (current_dB > cfg->db_keep_alive) {
                if (now - last_started > cfg->time_max) {
                    // max time reached
                    started_segmentation = false;
                    segmentation_finished = true;
                } else {
                    last_keep_alive = now;
                }
            } else {
                if (now - last_started > cfg->time_max) {
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
            last_started = now;
        }

        if (started_segmentation) {
            status = BaseSegmenter::SegmentationStatus::started;
        } else if (segmentation_finished){
            status = BaseSegmenter::SegmentationStatus::finished;
        } else {
            status = BaseSegmenter::SegmentationStatus::idle;
        }
    }

    ros::Time DoubleThresholdSegmenter::get_last_started(){
        return last_started;
    }
}
