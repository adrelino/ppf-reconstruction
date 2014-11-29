//
//  mainLocalCoordinates.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 26.10.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "LoadingSaving.h"
#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
#include "PPF.h"
#include <iostream>

using namespace std;

int main(int argc, char * argv[])
{
    PointCloud m=PointCloudManipulation::downSample(LoadingSaving::loadPointCloud("bunny/scene.xyz"),false);
    Quaternionf q=Quaternionf::Identity();
    q = q * AngleAxisf(deg2rad(10), Vector3f::UnitX());
    q = q * AngleAxisf(deg2rad(-20),Vector3f::UnitY());
    q = q * AngleAxisf(deg2rad(-123.333),Vector3f::UnitZ());


    Vector4f rot(q.x(),q.y(),q.z(),q.w());
    Vector3f tra(.1,300,-4);

    Isometry3f P = Translation3f(tra)*Quaternionf(rot);
    PointPairFeatures::printPose(P,"P_original:");
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

    PointPairFeatures::printPose(Pest,"P_est:");

    PointPairFeatures::err(P,Pest);




}

