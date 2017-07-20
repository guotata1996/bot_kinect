#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <vector>
#include <iostream>
#include <stdio.h>

#include "util.h"
#define MIN(a, b) ((a)<(b)?(a):(b))

//#define RGB
//#define IR
//#define DEPTH
//#define RECOGNITION
#define AVOID

int main(int argc , char** argv)
{
    /******************************************************************************/
    //launch kinect

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    if (freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    std::string serial = "";
    bool enable_rgb = true;
    bool enable_depth = true;

    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }

    pipeline = new libfreenect2::CpuPacketPipeline();
    dev = freenect2.openDevice(serial , pipeline);

    int type = libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    libfreenect2::SyncMultiFrameListener listener(type);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);


    if (!dev->start())
        return -1;
    libfreenect2::Freenect2Device::ColorCameraParams camera_params = dev->getColorCameraParams();

    while (true){
        if (!listener.waitForNewFrame(frames , 30 * 1000)) {
                    std::cout << "timeout!" << std::endl;
                    break;
        }
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];

#ifdef RGB
        unsigned char* rgbdata = rgb->data;
        CvSize size;
        size.height = 1080; size.width = 1920;

        char* BGRData = (char*)malloc(size.height*size.width*sizeof(char));
        for (int i = 0; i != size.height; ++i){
            for (int j = 0; j != size.width; ++j){
                int idx = i*size.width+j;
                char B = rgbdata[idx * 4];
                char G = rgbdata[idx * 4 + 1];
                char R = rgbdata[idx * 4 + 2];

                int h = getH(char2int(R),char2int(G),char2int(B));
                float s = getS(char2int(R),char2int(G),char2int(B));

                if ((abs(h-15)<15 || abs(h-345)<15) && s > 0.6)
                    BGRData[idx] = 255;
                else
                    BGRData[idx] = 0;
            }
        }

        IplImage* rgbimage = cvCreateImage(size,IPL_DEPTH_8U,1);

        cvSetData(rgbimage,BGRData, size.width);

        cvShowImage("Red Channel",rgbimage);
        cvWaitKey(100);

#endif

#ifdef IR
        unsigned char* irdata = ir->data;
        CvSize size;
        size.height = 424; size.width = 512;
        IplImage* irimage = cvCreateImage(size,IPL_DEPTH_8U,1);
        char* IRData = (char*)malloc(size.height*size.width*sizeof(char));

        for (int i = 0; i != size.height; ++i){
            for (int j = 0; j != size.width; ++j){
                int idx = i*size.width + j;
                int tmp = round(char2float(&(irdata[idx*4])) / 256.0);
                IRData[idx] = (char)tmp;
            }
        }

        cvSetData(irimage,IRData, size.width);
        cvShowImage("IR",irimage);
        cvWaitKey(100);
#endif

#ifdef DEPTH
        unsigned char* depthdata = depth->data;
        CvSize size;
        size.height = 424; size.width = 512;
        IplImage* depthImage = cvCreateImage(size,IPL_DEPTH_8U,1);
        char* DEPTHData = (char*)malloc(size.height*size.width*sizeof(char));

        for (int i = 0; i != size.height; ++i){
            for (int j = 0; j != size.width; ++j){
                int idx = i*size.width + j;
                char2f cf;
                cf.c[0] = depthdata[idx*4];
                cf.c[1] = depthdata[idx*4+1];
                cf.c[2] = depthdata[idx*4+2];
                cf.c[3] = depthdata[idx*4+3];
                int tmp = 255 - MIN(round(cf.f / 8.0),255);
                DEPTHData[idx] = (char)tmp;
            }
        }

        cvSetData(depthImage, DEPTHData, size.width);
        cvShowImage("DEPTH", depthImage);
        cvWaitKey(100);

#endif

#ifdef RECOGNITION
        libfreenect2::Registration *registration = new libfreenect2::Registration(dev->getIrCameraParams() , dev->getColorCameraParams());
        libfreenect2::Frame undistorted(512 , 424 , 4) , registered(512 , 424 , 4);
        registration->apply(rgb , depth , &undistorted , &registered);
        unsigned char* registereddata = registered.data;
        unsigned char* undistorteddata = undistorted.data;

        CvSize size;
        size.height = 424; size.width = 512;

        char* REGISTERData = (char*)malloc(size.height*size.width*sizeof(char));

        for (int i = 0; i != size.height; ++i){
            for (int j = 0; j != size.width; ++j){
                int idx = i*size.width+j;
                char B = registereddata[idx * 4];
                char G = registereddata[idx * 4 + 1];
                char R = registereddata[idx * 4 + 2];

                int h = getH(char2int(R),char2int(G),char2int(B));
                float s = getS(char2int(R),char2int(G),char2int(B));

                if ((abs(h - 360) < 15 || abs(h - 0) < 15) && s > 0.6)
                    REGISTERData[idx] = 255;
                else
                    REGISTERData[idx] = 0;
            }
        }

        IplImage* biimage = cvCreateImage(size,IPL_DEPTH_8U,1);
        //IplImage* depthImage = cvCreateImage(size,IPL_DEPTH_8U,1);

        cvSetData(biimage, REGISTERData, size.width);
        //cvSetData(depthImage, DEPTHData, size.width);

        CvMemStorage* storage = cvCreateMemStorage(0);
        CvContourScanner scanner = NULL;
        scanner = cvStartFindContours(biimage, storage, sizeof(CvContour), CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);

        CvRect rect;
        CvSeq* contour = NULL;
        double minarea = 180.0, tmparea = 0.0, maxarea = 0.0;
        int rec_x = 0, rec_y = 0;

        while ((contour = cvFindNextContour(scanner)) != NULL){
            tmparea = fabs(cvContourArea(contour));
            rect = cvBoundingRect(contour);

            if (tmparea > minarea && tmparea > maxarea){
                rec_x = rect.x + rect.width / 2;
                rec_y = rect.y + rect.height / 2;
            }
        }

        //fetch depth data

        if (rec_x != 0){
            int idx = rec_x + rec_y*size.width;
            float dep = char2float(&undistorteddata[idx*4]);
            if (dep != 0){
                int x = (rec_x - 282) * dep / camera_params.fx / 3.45;
                std::cout << "depth is" << dep - 50 << " ; x is " << x << std::endl;
            }
        }

#endif

#ifdef AVOID
        std::vector<float> ys,ds;
        float valid_ds[220];
        float angle2dist[15];
        
        unsigned char* depthdata = depth->data;
        CvSize size;
        size.height = 424; size.width = 512;
        IplImage* depthImage = cvCreateImage(size,IPL_DEPTH_8U,1);
        char* DEPTHData = (char*)malloc(size.height*size.width*sizeof(char));

        for (int j = 25; j <= 487; j += 33){ /*25 58 .. 487*/
        //for (int j = 490; j <= 490; ++j){ /*DEBUG*/
            ys.clear();
            ds.clear();

            for (int i = 0; i != size.height; ++i){
                int idx = i*size.width + j;
                char2f cf;
                cf.c[0] = depthdata[idx*4];
                cf.c[1] = depthdata[idx*4+1];
                cf.c[2] = depthdata[idx*4+2];
                cf.c[3] = depthdata[idx*4+3];

                ys.push_back(size.height - i);
                ds.push_back(1000.0 / cf.f);
            }

            int valid_len = 0;

             /*DEBUG*/
            //FILE* out = std::fopen("/home/guo/Documents/test/out_508.txt","w");
            for (int k = 0; k != ys.size(); ++k){
                if (ds.at(k) > 0.72 && ds.at(k) < 1.6 && ys.at(k) < 140){
                     /*DEBUG*/
                    //std::fprintf(out, "%f %f\n",ys.at(k),ds.at(k));

                    valid_ds[valid_len] = ds.at(k);
                    valid_len ++;
                }
            }

             /*DEBUG*/
            //std::fclose(out);

            histogram result = hist(valid_ds, valid_len);
            float mean = valid_len * 1.0 / RESOLUTION;
            float invdist = 0.0;


            for(int i = result.len - 1; i >=0; --i){
                if (result.cnt[i] > mean*2){
                    invdist = result.mid[i];
                    break;
                }
            }
            angle2dist[(j - 25)/33] = invdist == 0 ? 0 : 1 / invdist;
        }

        for (int i = 0; i != 15; ++i){
            if (angle2dist[i] > 0.1)
                std::cout << 7 - i << " " << angle2dist[i] << std::endl;
        }

        /*DEBUG*/
        break;

#endif

        listener.release(frames);
    }

}
