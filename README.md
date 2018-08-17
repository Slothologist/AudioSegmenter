# AudioSegmenter
Segmentation of audio for a speech pipeline using jack audio. 

Will read audio and write it back if...
   - The db_min is reached
   - The db does not fall below db_keep_alive for more than time_keep_alive ms
   - The audio is not longer than time_max ms

TODO's
   - check for repeated audio-fragment after segemtnation is finished
   - openmp the dB calculation in utils.cpp