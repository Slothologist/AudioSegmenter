//
// Created by rfeldhans on 02.10.18.
//

#ifndef AUDIO_SEGMENTER_BASE_SEGMENTER_H
#define AUDIO_SEGMENTER_BASE_SEGMENTER_H

#include <jack/jack.h>
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


        virtual void segment(jack_default_audio_sample_t* sample,
                             jack_nframes_t frame_count,
                             SegmentationStatus& status){}
        virtual ros::Time get_last_started(){}

    protected:
        utils::config* cfg;
        ros::Time last_started;
    };
}

#endif //AUDIO_SEGMENTER_BASE_SEGMENTER_H
