#include "kinect.h"

targetPos::targetPos(float d, int o){
    this->depth = d;
    this->offset = o;
}

Kinect::Kinect()
{
    libfreenect2::PacketPipeline *pipeline = 0;

    if (freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        exit(1);
    }

    std::string serial = "";

    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }

    pipeline = new libfreenect2::CpuPacketPipeline();
    dev = freenect2.openDevice(serial , pipeline);

    this->listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
    dev->setColorFrameListener(this->listener);
    dev->setIrAndDepthFrameListener(this->listener);

    if (!dev->start())
        exit(1);

    this->params = dev->getColorCameraParams();

}

targetPos Kinect::detectTarget(int color)
{
    libfreenect2::FrameMap frames;
    if (!listener->waitForNewFrame(frames , 30 * 1000)) {
          std::cout << "timeout!" << std::endl;
          listener->release(frames);
          return targetPos(0.0,0);
    }
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];
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

            if ((abs(h - 360 - color) < 15 || abs(h - color) < 15) && s > 0.6)
                REGISTERData[idx] = 255;
            else
                REGISTERData[idx] = 0;
        }
    }

    IplImage* biimage = cvCreateImage(size,IPL_DEPTH_8U,1);

    cvSetData(biimage, REGISTERData, size.width);

    CvMemStorage* storage = cvCreateMemStorage(0);
    CvContourScanner scanner = NULL;
    scanner = cvStartFindContours(biimage, storage, sizeof(CvContour), CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);

    CvRect rect;
    CvSeq* contour = NULL;
    double minarea = 100.0, tmparea = 0.0, maxarea = 0.0;
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
            int x = (rec_x - 282) * dep / params.fx / 3.45;
            std::cout << "depth is" << dep - 50 << " ; x is " << x << std::endl;
            listener->release(frames);
            return targetPos(dep - 50, x);
        }
    }
    listener->release(frames);
    return targetPos(0.0,0);
}

float* Kinect::detectObstacle(){
    libfreenect2::FrameMap frames;
    if (!listener->waitForNewFrame(frames , 30 * 1000)) {
           std::cout << "timeout!" << std::endl;
           listener->release(frames);
           return NULL;
    }
    libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];
    std::vector<float> ys,ds;
    float valid_ds[220];
    float* angle2dist = new float[15];

    unsigned char* depthdata = depth->data;
    CvSize size;
    size.height = 424; size.width = 512;
    IplImage* depthImage = cvCreateImage(size,IPL_DEPTH_8U,1);
    char* DEPTHData = (char*)malloc(size.height*size.width*sizeof(char));

    for (int j = 25; j <= 487; j += 33){ /*25 58 .. 487*/
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
        for (int k = 0; k != ys.size(); ++k){
            if (ds.at(k) > 0.72 && ds.at(k) < 1.6 && ys.at(k) < 140){
                valid_ds[valid_len] = ds.at(k);
                valid_len ++;
            }
        }


        histogram result = hist(valid_ds, valid_len);
        float mean = valid_len * 1.0 / RESOLUTION;
        float invdist = 0.0;


        for(int i = result.len - 1; i >=0; --i){
            if (result.cnt[i] > mean*2){
                invdist = result.mid[i];
                break;
            }
        }
        angle2dist[14 - (j - 25)/33] = invdist == 0 ? 0 : 1 / invdist;
    }
    listener->release(frames);
    return angle2dist;
}
