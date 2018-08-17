# AudioSegmenter
Segmentation of audio for a speech pipeline using jack audio. 

Will read audio and write it back if...
   - The db_min is reached
   - The db does not fall below db_keep_alive for more than time_keep_alive ms
   - The audio is not longer than time_max ms

TODO's
   - use only one channel for reading and writing
   - publish ros messages for info
      - start and end time of every segmented audio-scrap
      - in additional thread, to not clutter jackaudio