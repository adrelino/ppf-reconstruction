#include <gtest/gtest.h>
#include "testHelpers.h"

#include "LoadingSaving.h"
#include "PointCloudManipulation.h"

namespace{

using namespace std;

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
              //cout<< (*exp3)[i][j] <<endl;
              EXPECT_EQ((*exp3)[i][j],exp2[i*3+j]) << "rowcol equal";
          }
      }

}

TEST(Transformations,simple){
    Vector3f pt(0,1,2);

    Isometry3f P = Isometry3f::Identity();
    P.translation() = Vector3f(0,1,-2);

    Vector3f ptR=P*pt;


    Matrix3f threepts;
    threepts << 0,4,7
               ,1,5,8,
                2,6,9;


    Matrix3f ptR2=P*threepts;

    //cout<<ptR2<<endl;

    Vector3f pt2(4,5,6);


    vector<Vector3f> threeptsVec;
    threeptsVec.push_back(pt);
    threeptsVec.push_back(pt);
    threeptsVec.push_back(pt2);


    float* ptr = &threeptsVec[0][0];
    Map<Matrix3f> yeah(ptr);

    //cout<<yeah<<endl;

    //MatrixXf mat = MatrixXf::Zero(4,10);
    //mat(0,0)=pt(0);

    //cout<<"mat\n"<<mat<<endl;


    //cout<<"transformed\n"<<P.matrix()*mat<<endl;

}

//https://forum.kde.org/viewtopic.php?f=74&t=111955
//http://stackoverflow.com/questions/13722062/mesh-using-eigen-matrixx3f-or-stdvectorvector3f
TEST(NewLayout,basic){
    PointCloud pairBoth=LoadingSaving::loadPointCloud("bunny/model.xyz");
    vector<Vector3f> vec = pairBoth.pts;

    int N = vec.size();

    Map<Matrix<float,3,Dynamic,ColMajor> > mat(&vec[0].x(), 3, N);

    for(int i=0; i<N; i++){
        EXPECT_TRUE(mat.col(i).isApprox(vec[i]));
        //cout<<mat.col(i)<<vec[i]<<endl;
    }
}

TEST(NewLayout,transform){
    PointCloud C=LoadingSaving::loadPointCloud("bunny/model.xyz",1);

    //cout<<"C "<<C<<endl;

    Isometry3f P(PointCloudManipulation::getTranslationToCentroid(C));

    //cout<<"P"<<endl<<P.matrix()<<endl;

    PointCloud C2 = C.projected(P);

    //cout<<"C2 "<<C2<<endl;

    EXPECT_FLOAT_EQ(C2.nor[0].x(),C.nor[0].x()) << "Normals should be unnaffected by translation";
    EXPECT_FLOAT_EQ(C2.nor[0].y(),C.nor[0].y()) << "Normals should be unnaffected by translation";
    EXPECT_FLOAT_EQ(C2.nor[0].z(),C.nor[0].z()) << "Normals should be unnaffected by translation";

    EXPECT_TRUE(C2.pts[0].isApprox(Vector3f::Zero())) << "Mean of 1 pt in pt cloud should be set to 0";
}

TEST(NewLayout,transform2){
    PointCloud C=LoadingSaving::loadPointCloud("bunny/model.xyz");

    stringstream ss1;
    ss1<<"bunny/depth-poses/poses_"<<1<<".txt";
    Isometry3f P(LoadingSaving::loadMatrix4f(ss1.str()));

    PointCloud C2 = C.projected(P);
    PointCloud C3 = C2.projected(P.inverse());

    EXPECT_TRUE(vec2mat(C.pts).isApprox(vec2mat(C3.pts))) << "2 translation and its inverse should be identity ";
    EXPECT_TRUE(vec2mat(C.nor).isApprox(vec2mat(C3.nor))) << "same for normals";


}

TEST(NewLayout,downSample){
    PointCloud cloud=LoadingSaving::loadPointCloud("bunny/model.xyz",10);

    cloud=PointCloudManipulation::downSample(cloud,false);

}

TEST(NewLayout,normals){
    PointCloud cloud=LoadingSaving::loadPointCloud("bunny/bunny_smoothNormals.xyz");
    //cloud=PointCloudManipulation::downSample(cloud,false);

    vector<Vector3f> norGold = cloud.nor;

    for(float neighRadius=0.005f;neighRadius<0.05f;neighRadius+=0.005f){
        vector<Vector3f> nor=PointCloudManipulation::estimateNormals(cloud.pts,norGold,neighRadius);

        int numBad=0;
        for(int i=0; i< nor.size(); i++){
            double angle=acos(norGold[i].dot(nor[i]));
            if(angle>1){
                numBad++;
            }
        }

        cout<<"neighRadius: "<<neighRadius<< " #bad normals: "<<numBad<<"/"<<cloud.pts.size()<<endl;

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

TEST(NewLayout,ICP){
    PointCloud C=LoadingSaving::loadPointCloud("bunny/bunny_smoothNormals.xyz",100);

    Quaternionf q=Quaternionf::Identity();
    q = q * AngleAxisf(deg2rad(-1), Vector3f::UnitX());
    q = q * AngleAxisf(deg2rad(2),Vector3f::UnitY());
    q = q * AngleAxisf(deg2rad(-3),Vector3f::UnitZ());

    Vector4f rot(q.x(),q.y(),q.z(),q.w());
    //Vector4f rot(.2,.2,.2,.4);
    //Vector3f tra(0,0,0);//
    Vector3f tra(.01,-0.01,-0.005);//,0.5,0.01);

    Isometry3f P = Translation3f(tra)*Quaternionf(rot);
    printPose(P,"P_original:");
    PointCloud sSmall=C.projected(P);

    Isometry3f P_icp=ICP::getTransformationBetweenPointClouds(C,sSmall);

    printPose(P_icp,"P_icp:");

    float error=err(P,P_icp);

    EXPECT_LT(error,0.1f) << "Mean Pose error";


    //EXPECT_TRUE(P.data().isApprox(P_icp.data())) << "P_icp is not the original one";


}

} //namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


