#include <gtest/gtest.h>
#include "testHelpers.h"

#include "LoadingSaving.h"
#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
#include "PPF.h"

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace {



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

TEST(PPFs, PoseDiff){
    Isometry3f P1 = Translation3f(0, 0, 1) * AngleAxisf(M_PI, Vector3f::UnitX()+Vector3f::UnitY()*0.05);// * Scaling(s);
    Isometry3f P2 = Translation3f(0, 0, 0.9999) * AngleAxisf(M_PI, Vector3f::UnitX()-Vector3f::UnitY()*0.05);// * Scaling(s);
    Isometry3f P3 = Translation3f(0, 0, 1.0001) * AngleAxisf(M_PI+2, Vector3f::UnitX());// * Scaling(s);

    cout<<"P1:"<<endl;
    cout<<P1.linear().matrix()<<endl;

    cout<<"P2:"<<endl;
    cout<<P2.linear().matrix()<<endl;

    cout<<"P3:"<<endl;
    cout<<P3.linear().matrix()<<endl;

    cout<<"P1.equals(P2): "<<PointPairFeatures::isPoseSimilar(P1,P2)<<endl;
    cout<<"P1.equals(P3): "<<PointPairFeatures::isPoseSimilar(P1,P3)<<endl;
    cout<<"P2.equals(P3): "<<PointPairFeatures::isPoseSimilar(P2,P3)<<endl;

    Poses vec;
    vec.push_back(make_pair(P1,5));
    vec.push_back(make_pair(P2,10));
    vec.push_back(make_pair(P3,1));

    vector<Poses> clusters = PointPairFeatures::clusterPoses(vec);

    Poses posesAveraged = PointPairFeatures::averagePosesInClusters(clusters);

    cout<<"--averaged:"<<endl;
    printPoses(posesAveraged);
}


TEST(PPFs,ClusterAveraging){
    //adding small uniform/gaussian noise to quaternion and translation
    //http://www.mathworks.com/matlabcentral/fileexchange/40098-averaging-quaternions
    int nSamples = 1000;
    double sigma = 0.9;


    Quaternionf q(AngleAxisf(deg2rad(45), Vector3f::UnitX()));
    q = q * AngleAxisf(deg2rad(45),Vector3f::UnitY());

    Vector4f rot=Vector4f::Zero(); //(q.x(),q.y(),q.z(),q.w());
    Vector3f tra=Vector3f::Zero(); //0.1,0,0);

    Isometry3f P = Translation3f(tra)*Quaternionf(rot);

    Poses cluster;

    for (int i = 0; i < nSamples; ++i) {
        //Vector4f rotRand=RandomN::RandomNGet(4); //standard normally
        Vector4f rotRand=Vector4f::Random(); // uniform(-1,1) distributed samples
        //Vector4f rotTest = rot;
        //if(i % 3 == 1) rotTest=-rotTest;
        Vector4f rotNoisy = rot + sigma * rotRand;
        //Vector3f traRand=RandomN::RandomNGet(3);//standard normally
        Vector3f traRand=Vector3f::Random(); // uniform(-1,1) distributed samples
        Vector3f traNoisy = tra + sigma * traRand;

        //cout<<"traRand: "<<traRand.transpose()<<endl;
        //cout<<"rotRand: "<<rotRand.transpose()<<endl;

        Isometry3f Pi=Translation3f(traNoisy)*Quaternionf(rotNoisy);
        cluster.push_back(std::make_pair(Pi,1));
    }

    Pose PoseEst = PointPairFeatures::averagePosesInCluster(cluster);

    err(P,PoseEst);
}




}
