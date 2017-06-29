#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>


#include <iostream>

union char2f{
    char c[4];
    float f;
};

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


    while (true){
        if (!listener.waitForNewFrame(frames , 30 * 1000)) {
                    std::cout << "timeout!" << std::endl;
                    break;
        }
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];

        unsigned char* rgbdata = rgb->data;
        CvSize size;
        size.height = 1080; size.width = 1920;

        char* BGRData = (char*)malloc(size.height*size.width*sizeof(char));
        for (int i = 0; i != size.height; ++i){
            for (int j = 0; j != size.width; ++j){
                int idx = i*size.width+j;
                BGRData[idx] = rgbdata[idx * 4];
            }
        }

        IplImage* rgbimage = cvCreateImage(size,IPL_DEPTH_8U,1);

        cvSetData(rgbimage,BGRData, 1920);

        cvShowImage("Blue Channel",rgbimage);
        cvWaitKey(0);

        /*
        unsigned char* irdata = ir->data;
        CvSize size;
        size.height = 424; size.width = 512;
        IplImage* irimage = cvCreateImage(size,IPL_DEPTH_8U,1);
        char* IRData = (char*)malloc(size.height*size.width*sizeof(char));

        for (int i = 0; i != size.height; ++i){
            for (int j = 0; j != size.width; ++j){
                int idx = i*size.width + j;
                char2f cf;
                cf.c[0] = irdata[idx*4];
                cf.c[1] = irdata[idx*4+1];
                cf.c[2] = irdata[idx*4+2];
                cf.c[3] = irdata[idx*4+3];
                //std::cout << cf.f << std::endl;
                int tmp = round(cf.f / 256.0);
                IRData[idx] = (char)tmp;
            }
        }

        cvSetData(irimage,IRData, 512);
        cvShowImage("IR",irimage);
        cvWaitKey(0);

        */
        listener.release(frames);
    }

}
