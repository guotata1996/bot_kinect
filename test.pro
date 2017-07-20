TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    kinect.cpp \
    test.cpp

INCLUDEPATH += /home/guo/freenect2/include/
LIBS += -L/home/guo/freenect2/lib/ -lfreenect2
LIBS += -L/usr/local/lib/ -lopencv_core -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_photo
HEADERS += \
    kinect.h

