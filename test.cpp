#include <iostream>
#include "kinect.h"

#include "unistd.h"
using namespace std;

int main(){
    Kinect camera;
    while (true) {
        targetPos tar = camera.detectTarget();
        float* obs = camera.detectObstacle();

        cout << "target depth is " << tar.depth << " ,target offset is " << tar.offset << endl;
        for (int i = 0; i != 15; ++i){
            cout << i << " " << obs[i] << endl;
        }
        cout << endl << endl;
        sleep(3);
    }
}
