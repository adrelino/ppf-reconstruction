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
/*
TEST(PPFs,LocalCoordinates){
    PointCloud m=PointCloudManipulation::downSample(LoadingSaving::loadPointCloud("bunny/scene.xyz"),ddist);
    Quaternionf q=Quaternionf::Identity();
    q = q * AngleAxisf(deg2rad(10), Vector3f::UnitX());
    q = q * AngleAxisf(deg2rad(-20),Vector3f::UnitY());
    q = q * AngleAxisf(deg2rad(-123.333),Vector3f::UnitZ());


    Vector4f rot(q.x(),q.y(),q.z(),q.w());
    Vector3f tra(.1,300,-4);

    Isometry3f P = Translation3f(tra)*Quaternionf(rot);
    printPose(P,"P_original:");
    PointCloud s=PointCloudManipulation::projectPointsAndNormals(P, m);

    //now we simulate that there is a ppf correspondence between first row of sSmall and sSmallProjected

    //PPF:planarRotationAngle
    PPF ppfModel(m,0,1);  //also gets alpha-> planar rot angle to local coords
    PPF ppfScene(s,0,1);  //also gets alpha-> planar rot angle to local coords

    double alpha =PointPairFeatures::getAngleDiffMod2Pi(ppfModel.alpha, ppfScene.alpha);

    Vector3f s_m=s.pts[0];
    Vector3f s_n=s.nor[0];

    Vector3f m_m=m.pts[0];
    Vector3f m_n=m.nor[0];

    Isometry3f Pest = PointPairFeatures::alignSceneToModel(s_m,s_n,m_m,m_n,alpha);

    printPose(Pest,"P_est:");

    float error=err(P,Pest);
    EXPECT_NEAR(error,0,0.1f);
}*/

//TEST(Trans,inter){
//    Isometry3f P(LoadingSaving::loadMatrix4f("bunny/depth-poses/cloudToPLY-coarse_4.txt"));
//    Isometry3f Pn(LoadingSaving::loadMatrix4f("bunny/depth-poses/cloudToPLY-coarse_5.txt"));

//    Isometry3f P_inter=Pn*P.inverse();
//    Isometry3f P_est=P_inter*P;

//    err(Pn,P_est,true);

//}

TEST(Normals,nor){

}

}//end NS
