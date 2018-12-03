# AudioSegmenter
Segmentation of audio for a speech pipeline using jack audio.
Intended to use as a VAD (Voice Activation Detection) in a speech recognition pipeline.

## Disclaimer

The AudioSegmenter is in development and currently not recommended for use.
Requirements will most likely change in the near future; We will move away from jackaudio as sound framework to a more dedicated framework called esiaf, which is currently beeing worked on.

## Current Segmentation Methods
A basic interface allows for different VAD algorithms. At this point in time, only one is implemented.

### Double Threshold Segmenter

Will read audio and write it back if...
   - The db_min is reached
   - The db does not fall below db_keep_alive for more than time_keep_alive ms
   - The audio is not longer than time_max ms

Otherwise, it will output a zero signal.

## Requirements

- Jack audio

- Somewhat recent gcc (5.4 is tested, but it should theoretically work from 4.8 onwards)