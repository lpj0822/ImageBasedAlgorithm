#ifndef BASE_IMAGE_FILTER_H
#define BASE_IMAGE_FILTER_H

void gaussianSmoothGray(const unsigned char* src, const int width, const int height,
    const float *kernel, const int kernelSize, unsigned char* dst);

#endif // BASE_IMAGE_FILTER_H
