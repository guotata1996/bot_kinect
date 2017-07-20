#ifndef KINECT_H
#define KINECT_H
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <vector>

#include <math.h>
#include <opencv2/opencv.hpp>
#include <stdarg.h>
#include <map>

#include <iostream>

#define RESOLUTION 10

struct histogram{
    int len = RESOLUTION;
    float mid[RESOLUTION];
    int cnt[RESOLUTION];
};

union charint{
    char c[4];
    int i;
};

union char2f{
    char c[4];
    float f;
};

struct targetPos{
    targetPos(float d, int o);
    float depth;
    int offset;
};

class Kinect
{
public:
    Kinect();
    /*
     * @params
     * color: H value of hsv space
     * now supported: red(0) blue(200) yellow(60)
     * @return
     * depth in millimeters, offset in centimeters
     * if no target detected, targetPos.depth = 0*/
    targetPos detectTarget(int color);

    /*
     * @return
     * in meters */
    float* detectObstacle();
private:
    libfreenect2::SyncMultiFrameListener* listener;
    libfreenect2::Freenect2Device::ColorCameraParams params;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::FrameMap frames;
    libfreenect2::Freenect2 freenect2;

    histogram hist(float* data, int data_len){
        float min = 9999, max = 0;
        for (int i = 0; i != data_len; ++i){
            if (data[i] > max)
                max = data[i];
            if (data[i] < min)
                min = data[i];
        }
        float precision = (max - min)/RESOLUTION;

        histogram res;
        for (int i = 0; i != RESOLUTION; ++i){
            res.cnt[i] = 0;
        }

        for (int i = 0; i != RESOLUTION; ++i){
            res.mid[i] = min + precision * i + precision / 2;
        }
        for (int i = 0; i != data_len; ++i){
            int idx = floor((data[i] - min)/precision);
            if (idx == RESOLUTION)
                idx--;
            res.cnt[idx] ++;
        }
        return res;
    }

    float getS(int r, int g, int b){
        int max = r, min = r;
        if (g>max)
            max = g;
        if (b>max)
            max = b;
        if (g<min)
            min = g;
        if (b<min)
            min = b;
        return (max - min)*1.0/max;
    }

    int getH(int r, int g, int b){
        int max = r, min = r,h;
        if (g>max)
            max = g;
        if (b>max)
            max = b;
        if (g<min)
            min = g;
        if (b<min)
            min = b;
        if (r == max){
            h = round((g-b)*60.0/(max - min));
        }
        if (g == max){
            h = round(120+(b-r)*60.0/(max - min));
        }
        if (b == max){
            h = round(240+(r-g)*60.0/(max - min));
        }
        if (h<0)
            h += 360;
        return h;
    }

    float char2float(unsigned char* x){
        char2f cf;
        cf.c[0] = x[0];
        cf.c[1] = x[1];
        cf.c[2] = x[2];
        cf.c[3] = x[3];
        return cf.f;
    }

    int char2int(char x){
        charint ci;
        ci.c[0] = x;
        ci.c[1] = 0;
        ci.c[2] = 0;
        ci.c[3] = 0;
        return ci.i;
    }
};

#endif // KINECT_H
