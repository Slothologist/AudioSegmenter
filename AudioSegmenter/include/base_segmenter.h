//
// Created by rfeldhans on 02.10.18.
//

#ifndef AUDIO_SEGMENTER_BASE_SEGMENTER_H
#define AUDIO_SEGMENTER_BASE_SEGMENTER_H

#include "ros/ros.h"
#include "utils.h"

namespace segmenter {
    class BaseSegmenter{

    public:

        enum SegmentationStatus {
            idle,
            started,
            finished
        };

        explicit BaseSegmenter(utils::config* cfg):
                cfg(cfg)
        {};


        virtual void segment(float* sample,
                             size_t frame_count,
                             SegmentationStatus& status){}
        virtual ros::Time get_last_started(){}

    protected:
        utils::config* cfg;
        ros::Time last_started;
    };
}

#endif //AUDIO_SEGMENTER_BASE_SEGMENTER_H
