#ifndef IMAGE_PROCESSING
#define IMAGE_PROCESSING

void computeDerivativeX(const unsigned char *src, int width, int height, short *dst);

void computeDerivativeY(const unsigned char *src, int width, int height, short *dst);

//Helper function that integrates an image
void computeIntegrate(const unsigned char *image, const int imageWidth, const int imageHeight, unsigned int *integralImage);

//Helper function that integrates an image
void integrate(const unsigned char *image, const int imageWidth, const int imageHeight, unsigned int *integralImage);

#endif // IMAGE_PROCESSING

