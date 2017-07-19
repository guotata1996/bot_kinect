#include <iostream>
#include "util.h"
using namespace std;

int main(){
    histogram output;
    float mydata[25] = {0.0, 3.1,15.2,3.3,4.5,8.9,9.1,14.2,13.6,11.1,1.8,7,6,19.1, 19.9, 12.1,10.8, 10.7, 12.3, 14.4, 15.8, 16.8, 13, 6.6,5.4};
    output = hist(mydata, 25);
    return 0;
}
