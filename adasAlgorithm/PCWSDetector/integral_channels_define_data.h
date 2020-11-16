#ifndef INTEGRAL_CHANNELS_DEFINE_DATA_H
#define INTEGRAL_CHANNELS_DEFINE_DATA_H

#define NUMBER_BINS 6  //Number of angle bins used when computing the HOG channels. Currently supported values are 6 or 18.

#define NUMBER_CHANNELS 10  // 6 gradients orientations, 1 gradient intensity, 3 LUV color channels

#define SHRINKING_FACTOR 4 //how much we shrink the channel images?default: 4 is the value that we use for most of our experiments.

#define INPUT_TO_CHANNEL_SCALE 0.25f //input image scale to the channel of parameter (1.0f / SHRINKING_FACTOR)

#endif // INTEGRAL_CHANNELS_DEFINE_DATA_H

