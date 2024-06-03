#pragma once


#include <vector>
using namespace std;


typedef unsigned char       BYTE;


class LUT
{
public:
    double metersPerPixel;

   // int *plut;
	vector<int> vtlut;
	int length;
};

;

class GridMap
{

public:
    double x0, y0;        // minimum x, y (lower-left corner of lower-left pixel);
    double metersPerPixel;

    int    width, height; // in pixels. Always a multiple of four.
    unsigned char   *data;

    double range;

    unsigned char   defaultFill;
public:

    GridMap(void);

    ~GridMap(void);


    void makeMeters(double dx0, double dy0, double sizex, double sizey, double dmetersPerPixel, int ndefaultFill);

    void makePixels(double dx0, double dy0, int nwidth, int nheight, double dmetersPerPixel, int ndefaultFill, bool broundUpDimensions);

    void fill(int v);

    int clamp(int v, int min, int max);

    void makeGaussianLUT(double scale, double cliffDistMeters, double expDecayMSq, LUT &lut);

    void drawDot(double x, double y, LUT& lut,int lutlength);

    void drawRectangle(double cx, double cy,
                              double x_size, double y_size,
                              double theta,
                              LUT& lut,int lutlength);


    void setValue(double x, double y, BYTE v);

    void setValueIndex(int ix, int iy, BYTE v);

    void setValueIndexSafe(int ix, int iy, BYTE v);

    bool saveMap(const char* fileName);

};

