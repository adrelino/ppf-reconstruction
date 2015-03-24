#include "Params.h"
#include "math.h"

using namespace std;

namespace{
    static cv::Mat linspace(int nCols)//float x0, float x1, int n)
    {
        cv::Mat pts(256, nCols, CV_8UC1);
        //float step = (x1-x0)/(n-1);
        for(int i = 0; i <= 255; i++)
            pts.row(i) = i;
        return pts;
    }
}

Params* Params::instance = 0;


Params* Params::getInstance(){
    if(instance == 0){
        instance = new Params();
        instance->grayMap = linspace(1);
        cv::applyColorMap(instance->grayMap,instance->colorMap,cv::COLORMAP_JET);
    }
    return instance;
}

Eigen::Vector3f Params::colorJet(float val, float min, float max){
    float range = max-min;
    int idx = ((val-min)/range)*255.0;
    if(idx>255) idx=255;
    if(idx<0) idx=0;

    cv::Vec3b bgr = colorMap.at<cv::Vec3b>(idx,0);

    Eigen::Vector3f color(bgr[2],bgr[1],bgr[0]);

    //cout<<"colormapped "<< colorMap<<endl; //OpenCVHelpers::getImageType(faa.type())

    return color / 255.0f;

    //cv::imshow("test",foo);
    //cv::imshow("test2",faa);
    //cout<<"gray "<<OpenCVHelpers::getImageType(foo.type())<< foo<<endl;
}
