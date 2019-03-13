//
// Created by rfeldhans on 02.10.18.
//

#ifndef AUDIO_SEGMENTER_DOUBLE_THRESHOLD_SEGMENTER_H
#define AUDIO_SEGMENTER_DOUBLE_THRESHOLD_SEGMENTER_H

#include "base_segmenter.h"
#include "../include/utils.h"

namespace segmenter{
    class DoubleThresholdSegmenter : public BaseSegmenter{
    public:

        explicit DoubleThresholdSegmenter(utils::config* cfg);

        void segment(float*, size_t, SegmentationStatus&);

        ros::Time get_last_started();

    private:
        ros::Time last_started;
        ros::Time last_keep_alive;
        bool started_segmentation;
        bool segmentation_finished;
    };
}

#endif //AUDIO_SEGMENTER_DOUBLE_THRESHOLD_SEGMENTER_H
