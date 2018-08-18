# AudioSegmenter
Segmentation of audio for a speech pipeline using jack audio. 

Will read audio and write it back if...
   - The db_min is reached
   - The db does not fall below db_keep_alive for more than time_keep_alive ms
   - The audio is not longer than time_max ms
   
Check out the parent repository for more ros audio nodes: https://github.com/Slothologist/SpeechRecPipeLine

TODO's
   - check for repeated audio-fragment after segmentation is finished