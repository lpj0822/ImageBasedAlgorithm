#ifndef SOBEL_H
#define SOBEL_H

void sobelX(const unsigned char *src, int width, int height, short *dst);
void sobelY(const unsigned char *src, int width, int height, short *dst);

void prefilterXSobel(const unsigned char* src, const int width, const int height, unsigned char* dxImage);
void prefilterYSobel(const unsigned char *src, const int width, const int height, unsigned char *dyImage);

#endif // SOBEL_H

