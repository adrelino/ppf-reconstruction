#ifndef OPENCVHELPERS_H
#define OPENCVHELPERS_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <sstream>
#include <iostream>

#include <string>
#include <iostream>

namespace OpenCVHelpers {
using namespace std;
using namespace cv;

void showImage(const string &wndTitle, const Mat& img){
    cv::namedWindow(wndTitle, WINDOW_AUTOSIZE);
    cv::imshow(wndTitle.c_str(), img);
}

void imagesc(std::string title, cv::Mat mat,InputArray mask = noArray()) {
  double min,max;
  cv::minMaxLoc(mat,&min,&max, 0, 0, mask);

  Mat scaled = mat;
  //Mat meanCols;
  //reduce(mat, meanCols, 0, REDUCE_AVG );

  //Mat mean;
  //reduce(meanCols, mean, 1, REDUCE_AVG);

  //cout << "Max value: " << max << endl;
  //cout << "Mean value: " << mean.at<float>(0) << endl;
  //cout << "Min value: " << min << endl;

  if (std::abs(max) > 0.0000001f)
    scaled /= max;

  showImage(title, scaled);
}


void showDepthImage(const string &wndTitle, const Mat& img, InputArray mask = noArray(),bool print=false) {
  double min, max;
  minMaxIdx(img, &min, &max, 0, 0, mask);

  if(print) cout<<"    showDepthImage scaled:  min: "<<min<<" max:"<<max;




  //Mat img2=img*mask;
  Mat depthMap;

  float scale = 255.0f / (max - min);
  img.convertTo(depthMap, CV_8UC1, scale, -min*scale);


  Mat heatMap;
  applyColorMap(depthMap, heatMap, cv::COLORMAP_JET);

  cv::namedWindow(wndTitle, WINDOW_AUTOSIZE);
  cv::imshow(wndTitle.c_str(), heatMap);
}

std::string getImageType(int imgTypeInt)
{
    int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)

    int enum_ints[] =       {CV_8U,  CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4,
                             CV_8S,  CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4,
                             CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
                             CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
                             CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
                             CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
                             CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};

    string enum_strings[] = {"CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
                             "CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
                             "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
                             "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
                             "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
                             "CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
                             "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

    for(int i=0; i<numImgTypes; i++)
    {
        if(imgTypeInt == enum_ints[i]) return enum_strings[i];
    }
    return "unknown image type";
}

}

#endif // OPENCVHELPERS_H
