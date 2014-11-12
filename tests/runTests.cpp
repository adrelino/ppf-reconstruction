#include <gtest/gtest.h>
//#include <opencv2/core/core.hpp>

// our own testing code
#include "testHelpers.h"

#include "../src/LoadingSaving.h"
#include "../src/PointPairFeatures.h"

namespace{

using namespace std;
//using namespace cv;
//using namespace Eigen;

//TEST(LoadingSaving, cv) {
//  float A[3][4] = {
//    {47.000000000, 14.000000000, 9.000000000, 16.000000000},
//    {38.000000000, 39.000000000, 37.000000000, 45.000000000},
//    {45.000000000, 42.000000000, 34.000000000, 27.000000000},
//  };
//  Mat mA = Mat(3, 4, CV_32FC1, &A);

//  //mA[0][0] -=1;

//  float expectedRes[3][4] = {
//    {47.000000000, 14.000000000, 9.000000000, 16.000000000},
//    {38.000000000, 39.000000000, 37.000000000, 45.000000000},
//    {45.000000000, 42.000000000, 34.000000000, 27.000000000},
//  };
//  Mat mExpectedRes = Mat(3, 4, CV_32FC1, &expectedRes);
//  //cout<<mA<<endl;
//  areMatricesEqual(mExpectedRes, mA);
//}

TEST(Eigen, SymmetricMatrixTranspose) {
    Eigen::Matrix2i A, B;

    A << 1, 3,
         3, 4;
    B = A.transpose();

    EXPECT_EQ_MATRIX(A,B);

}

TEST(LoadingSaving, MatrixXf) {
  MatrixXf mA = MatrixXf::Random(5,5);
  LoadingSaving::saveMatrixXf("MatrixXf",mA);
  MatrixXf mB=LoadingSaving::loadMatrixXf("MatrixXf");
  EXPECT_FLOAT_EQ_MATRIX(mA, mB)
}

TEST(LoadingSaving, MatrixXd) {
  MatrixXd mA = MatrixXd::Random(5,5);
  LoadingSaving::saveMatrixXd("MatrixXd",mA);
  MatrixXd mB=LoadingSaving::loadMatrixXd("MatrixXd");
  EXPECT_DOUBLE_EQ_MATRIX(mA,mB)
}

TEST(LoadingSaving, MatrixXi) {
  MatrixXi mA = MatrixXi::Random(100,2);
  LoadingSaving::saveMatrixXi("MatrixXi",mA);
  MatrixXi mB=LoadingSaving::loadMatrixXi("MatrixXi");
  EXPECT_EQ_MATRIX(mA, mB);
}

TEST(LoadingSaving, Matrix4f) {
  Matrix4f mA = Matrix4f::Random();
  LoadingSaving::saveMatrixXf("Matrix4f",mA);
  Matrix4f mB=LoadingSaving::loadMatrix4f("Matrix4f");
  EXPECT_FLOAT_EQ_MATRIX(mA,mB)
}

TEST(Quaternion, test_avg_quat) {
  //cout<<'Testing un-weighted quaternion averaging'<<endl;
  // Average 100 times
  int numTrials = 100;
  int perturb = 5;

  double errNaiveSum = 0;
  double errMarkleySum = 0;

  for(int i=0; i<numTrials; i++){
      Vector4f qinit = Vector4f::Random();
      qinit = qinit.normalized();

      Poses poses;
      for(int j = 0; j< 10; j++){
          Vector4f q2(qinit);
          q2 += Vector4f::Random()*perturb/2.0; //matlab : 0->1, eigen -1->1
          poses.push_back(std::make_pair(Isometry3f(Quaternionf(q2)),1));
      }

      Quaternionf Qavg = PointPairFeatures::avg_quaternion_markleyQ(poses);

      //cout<<Qavg.coeffs()<<endl;
  }

}

//TEST(Quaternion, simple){
//    MatrixXf Q2 = LoadingSaving::loadMatrixXf("Q2");
//    Vector4f Qavg = PointPairFeatures::avg_quaternion_markley(Q2.adjoint());

//    Vector4f QavgExp(LoadingSaving::loadMatrixXf("Qavg").data());

//    if(Qavg(0)*QavgExp(0)<0){
//        Qavg*=-1;
//    }

//    cout<<Qavg.transpose()<<endl;
//    cout<<QavgExp.transpose()<<endl;


//    EXPECT_NEAR_MATRIX(Qavg.cwiseAbs(), QavgExp.cwiseAbs(),1e-2);
//}

//TEST(Quaternion, poses){
//    MatrixXf Q2 = LoadingSaving::loadMatrixXf("Q2");

//    Poses poses;
//    for(int j = 0; j< Q2.cols(); j++){
//       Vector4f q2(Q2.col(j).cast<double>());
//       poses.push_back(std::make_pair(Isometry3f(Quaternionf(q2)),1));
//    }
//    Vector4f Qavg = PointPairFeatures::avg_quaternion_markley(poses);

//    Vector4f QavgExp(LoadingSaving::loadMatrixXf("Qavg").data());

//    if(Qavg(0)*QavgExp(0)<0){
//        Qavg*=-1;
//    }

//    cout<<Qavg.transpose()<<endl;
//    cout<<QavgExp.transpose()<<endl;


//    areMatricesEqual<float>(Qavg.cwiseAbs(), QavgExp.cwiseAbs(),1e-2);
//}

//TEST(EigenValues,eigValues){
//    MatrixXd A = LoadingSaving::loadMatrixXd("A");

//    SelfAdjointEigenSolver<MatrixXd> eig(A);

//    //From eig, you have access to the sorted (increasing order) eigenvalues (eig.eigenvalues()) and respective eigenvectors (eig.eigenvectors()). For instance, eig.eigenvectors().rightCols(N) gives you the best N-dimension basis.
//    cout<<"A"<<endl<<A<<endl;
//    cout<<"eigvalues"<<endl<<eig.eigenvalues()<<endl;
//    cout<<"eigvectors"<<endl<<eig.eigenvectors()<<endl;
//}

TEST(two2arr,indexing){
      int expectedRes[3][3] = {
        {1,2,3},
        {4,5,6},
        {7,8,9},
      };

      int* exp2 = (int*) expectedRes;

      //http://stackoverflow.com/questions/14808908/c-pointer-to-two-dimensional-array

      //int** exp3 = (int**) exp2[0];
      int (*exp3)[3][3];
      exp3 = &expectedRes;
      //int exp4[][] = (*exp3);

      for(int i=0; i<3;i++){
          for(int j=0; j<3;j++){
              EXPECT_EQ( exp2[3*i+j],(*exp3)[i][j] );
          }
      }

}

TEST(floating,numnber){

    float b41=0.10793996f;
    //cout.precision(8);//numeric_limits<float>::digits10);
    //cout<<fixed<<b41<<endl;

    std::stringstream ss;
    ss.precision(8);
    ss<<b41;

    //cout<<ss.str()<<endl;

    float b41Test;
    ss>>b41Test;

    //cout<<b41Test<<endl;

    EXPECT_FLOAT_EQ(b41,b41Test);
}

} //namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


