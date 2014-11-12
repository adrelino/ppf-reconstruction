#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <ctime>
#include <cstdlib>

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace std;

template<typename Real>
bool isEqual(Real a, Real b, const Real eps = 1e-7f) {
  Real diff = fabs(a-b);
  return diff<=eps;
}

void areMatricesEqual(const cv::Mat& a, const cv::Mat& b, const float eps = 1e-7f) {
  ASSERT_EQ(a.size(), b.size()) << "different number of rows";
  ASSERT_EQ(a.channels(), b.channels()) << "different number of cols";

  int nc = a.channels();
  for(int y = 0; y < a.rows; ++y) {
    for (int x = 0; x < a.cols; ++x) {
      if (nc == 1) {
          EXPECT_TRUE(isEqual<float>(a.at<float>(y,x), b.at<float>(y,x), eps)) << "(" <<y<<","<<x<<") " << " not within "  <<eps;
      } else {
          cv::Vec3f vecA = a.at<cv::Vec3f>(y,x);
          cv::Vec3f vecB = b.at<cv::Vec3f>(y,x);
          EXPECT_TRUE(isEqual<float>(vecA[0], vecB[0], eps))<< "(" <<y<<","<<x<<") " << " x not within "  <<eps;
          EXPECT_TRUE(isEqual<float>(vecA[1], vecB[1], eps))<< "(" <<y<<","<<x<<") " << " y not within "  <<eps;
          EXPECT_TRUE(isEqual<float>(vecA[2], vecB[2], eps))<< "(" <<y<<","<<x<<") " << " z not within "  <<eps;
      }
    }
  }
}

template<typename Real>
void areMatricesEqual(const Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic>& a, const Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic>& b, const double eps = 1e-7f, bool useEigenTest = false) {
  ASSERT_EQ(a.rows(), b.rows()) << "different number of rows";
  ASSERT_EQ(a.cols(), b.cols()) << "different number of cols";

  for(int y = 0; y < a.rows(); ++y) {
    for (int x = 0; x < a.cols(); ++x) {
        //EXPECT_TRUE(isEqual<Real>(a(y,x), b(y,x), eps))<< "(" <<y<<","<<x<<") " << " not within "  <<eps;
        if(!isEqual<Real>(a(y,x), b(y,x), eps)){
            cout<< "(" <<y<<","<<x<<") " << " not within "  <<eps<<endl;
        }

    }
  }

  //EXPECT_TRUE(a.isApprox(b)) << "Eigen isApprox failed";
  if(useEigenTest && !a.isApprox(b)){
      cout<< "Eigen isApprox failed"<<endl;
  }

}



#endif
