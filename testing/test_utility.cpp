#include "test_utility.h"

void logMat(const char* fileName, const cv::Mat image)
{
    FILE *writeFile = fopen(fileName, "w");
    int i = 0;
    int j = 0;
    int k = 0;
    int offset = 0;
    char data[64];
    for (i = 0; i < image.rows; i++)
    {
        const uchar *imageData = image.ptr<uchar>(i);
        for (j = 0; j < image.cols; j++)
        {
            for(k = 0; k < image.channels(); k++, imageData++)
            {
                offset = sprintf(data, "%d ", *(imageData));
                fwrite(data, sizeof(char), offset, writeFile);
            }
        }
        fwrite("\n", sizeof(char), 1, writeFile);
    }
    fclose(writeFile);
}

void logImageU(const char* fileName, const unsigned char *img, const int width, const int height)
{
    FILE *writeFile = fopen(fileName, "w");
    size_t i = 0;
    size_t j = 0;
    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
            fprintf(writeFile, "%d ", img[j]);
        }
        fputc('\n', writeFile);
        img += width;
    }
    fclose(writeFile);
}

void logPoint(const char* fileName, const Point *point, const int count)
{
    FILE *writeFile = fopen(fileName, "w");
    int loop = 0;
    int offset = 0;
    char data[64];
    for (loop = 0; loop < count; loop++)
    {
        offset = sprintf(data, "%d,%d\n", point[loop].x, point[loop].y);
        fwrite(data, sizeof(char), offset, writeFile);
    }
    fclose(writeFile);
}

void logFPoint(const char* fileName, const FloatPoint *cornerPoint, const int count)
{
    FILE *writeFile = fopen(fileName, "w");
    int loop = 0;
    int offset = 0;
    char data[64];
    for (loop = 0; loop < count; loop++)
    {
        offset = sprintf(data, "%f,%f\n", cornerPoint[loop].x, cornerPoint[loop].y);
        fwrite(data, sizeof(char), offset, writeFile);
    }
    fclose(writeFile);
}

int matWrite(const cv::Mat &img, const std::string &fileName)
{
  std::ofstream output;
  output.open(fileName.c_str(), std::ofstream::binary);
  if (!output.is_open())
  {
    std::cerr << "failed to open the file : " << fileName << std::endl;
    return 0;
  }
  for (int r = 0; r < img.rows; r++)
  {
    const uchar* data = img.ptr<uchar>(r);
    output.write(reinterpret_cast<const char*>(data), sizeof(uchar) * img.cols);
  }
  output.close();
  return 1;
}

int readImage(const char *fileName, unsigned char *image)
{
    int count = 0;
    int size = 0;
    FILE *infile = NULL;
    infile = fopen(fileName, "rb");
    unsigned char buf[1024];

    if(infile == NULL)
    {
        //LOGI("Optical", "%s not exit\n", fileName);
        printf("%s, %s", fileName, "not exit\n");
        return -1;
    }
    while(1)
    {
        size = fread(buf, sizeof(unsigned char), 1024, infile);
        if(size == 0)
            break;
        memcpy(image + count, buf, size * sizeof(unsigned char));
        count += size;
    }

    fclose(infile);
    return count;
}
