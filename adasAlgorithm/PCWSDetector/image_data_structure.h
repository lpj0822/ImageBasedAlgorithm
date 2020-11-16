#ifndef IMAGE_DATA_STRUCTURE_H
#define IMAGE_DATA_STRUCTURE_H

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

typedef struct Point
{
    int x;
    int y;
}Point;

typedef struct Rect
{
    int minX;
    int minY;
    int maxX;
    int maxY;
}Rect;

typedef struct Point Size;

#endif // IMAGE_DATA_STRUCTURE_H

