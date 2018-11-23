#ifndef IMAGE_DATA_STRUCTURE_H
#define IMAGE_DATA_STRUCTURE_H

//color space
typedef struct RGB
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
}ElementRGB;

typedef struct LUV
{
    unsigned char l;
    unsigned char u;
    unsigned char v;
}ElementLUV;

typedef struct LAB
{
    unsigned char l;
    unsigned char a;
    unsigned char b;
}ElementLAB;

//base
typedef struct Point
{
    int x;
    int y;
}Point;

typedef struct FloatPoint
{
    float x;
    float y;
}FloatPoint;

typedef struct Size
{
    int width;
    int height;
}Size;

typedef struct Rectangle
{
    int x;
    int y;
    int width;
    int height;
}Rectangle;

typedef struct MyRectangle
{
    int minX;
    int minY;
    int maxX;
    int maxY;
}MyRectangle;

typedef struct ResultRect
{
    int x;
    int y;
    int width;
    int height;
    int confidence;
}ResultRect;

#endif // IMAGE_DATA_STRUCTURE_H

