#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>

//EIGEN Matrix GTEST Addons

//http://stackoverflow.com/questions/25146997/teach-google-test-how-to-print-eigen-matrix
template <class Base>
class EigenPrintWrap : public Base {
    friend void PrintTo(const EigenPrintWrap &m, ::std::ostream *o) {
        *o << "\n" << m;
    }
};

template <class Base>
const EigenPrintWrap<Base> &EigenMatrix(const Base &base) {
    return static_cast<const EigenPrintWrap<Base> &>(base);
}



//nice defines for EXPECT_[   ]_MATRIX

#define LOOP(a,b,comparator) \
    for(int row = 0; row < a.rows(); ++row) { \
        for (int col = 0; col < a.cols(); ++col) { \
            comparator(a(row,col), b(row,col)) << "(row=" <<row<<","<<"col="<<col<<")"; \
        } \
    } \

#define EXPECT_EQ_MATRIX(a,b) EXPECT_EQ(EigenMatrix(a), EigenMatrix(b))
#define EXPECT_EQ_MATRIX_V(a,b) EXPECT_EQ_MATRIX(a, b); LOOP(a,b,EXPECT_EQ)

//TODO: check if b is zero matrix:
//http://eigen.tuxfamily.org/dox/classEigen_1_1DenseBase.html#a158c2184951e6e415c2e9b98db8e8966
//internal::isMuchSmallerThan(const RealScalar&, RealScalar) instead.
#define EXPECT_NEAR_MATRIX(a,b,eps) \
    ASSERT_EQ(a.rows(), b.rows()) << "#rows don't match"; \
    ASSERT_EQ(a.cols(), b.cols()) << "#cols don't match"; \
    EXPECT_TRUE(a.isApprox(b,eps)) << "Eigen test failed: a is not approx b with eps="<<eps <<"  Actual:\n"<<EigenMatrix(a)<<"\n Expected:"<<EigenMatrix(b);
#define EXPECT_NEAR_MATRIX_V(a,b,eps) EXPECT_NEAR_MATRIX(a,b,eps); LOOP(a,b,EXPECT_NEAR)

#define EXPECT_FLOAT_EQ_MATRIX(a,b) EXPECT_NEAR_MATRIX(a,b,std::numeric_limits<float>::epsilon())// 1e-7f) {)
#define EXPECT_FLOAT_EQ_MATRIX_V(a,b) EXPECT_FLOAT_EQ_MATRIX(a,b); LOOP(a,b,EXPECT_FLOAT_EQ)
#define EXPECT_DOUBLE_EQ_MATRIX(a,b) EXPECT_NEAR_MATRIX(a,b,std::numeric_limits<double>::epsilon())// 1e-16f) {)
#define EXPECT_DOUBLE_EQ_MATRIX_V(a,b) EXPECT_DOUBLE_EQ_MATRIX(a,b); LOOP(a,b,EXPECT_DOUBLE_EQ)


template<typename Real>
bool isEqual(Real a, Real b, const Real eps = 1e-7f) {
  Real diff = fabs(a-b);
  return diff<=eps;
}



//openCV addons, makes GTest crash if used

//#include <opencv2/core/core.hpp>
//void areMatricesEqual(const cv::Mat& a, const cv::Mat& b, const float eps = 1e-7f) {
//  ASSERT_EQ(a.size(), b.size()) << "different number of rows";
//  ASSERT_EQ(a.channels(), b.channels()) << "different number of cols";

//  int nc = a.channels();
//  for(int y = 0; y < a.rows; ++y) {
//    for (int x = 0; x < a.cols; ++x) {
//      if (nc == 1) {
//          EXPECT_TRUE(isEqual<float>(a.at<float>(y,x), b.at<float>(y,x), eps)) << "(" <<y<<","<<x<<") " << " not within "  <<eps;
//      } else {
//          cv::Vec3f vecA = a.at<cv::Vec3f>(y,x);
//          cv::Vec3f vecB = b.at<cv::Vec3f>(y,x);
//          EXPECT_TRUE(isEqual<float>(vecA[0], vecB[0], eps))<< "(" <<y<<","<<x<<") " << " x not within "  <<eps;
//          EXPECT_TRUE(isEqual<float>(vecA[1], vecB[1], eps))<< "(" <<y<<","<<x<<") " << " y not within "  <<eps;
//          EXPECT_TRUE(isEqual<float>(vecA[2], vecB[2], eps))<< "(" <<y<<","<<x<<") " << " z not within "  <<eps;
//      }
//    }
//  }
//}



#endif
