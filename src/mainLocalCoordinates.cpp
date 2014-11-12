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
    MatrixXf m=PointCloudManipulation::downSample(LoadingSaving::loadMatrixXf("bunny/scene.xyz"),false);
    Quaternionf q=Quaternionf::Identity();
    q = q * AngleAxisf(deg2rad(10), Vector3f::UnitX());
    q = q * AngleAxisf(deg2rad(-20),Vector3f::UnitY());
    q = q * AngleAxisf(deg2rad(-123.333),Vector3f::UnitZ());


    Vector4f rot(q.x(),q.y(),q.z(),q.w());
    Vector3f tra(.1,300,-4);

    Isometry3f P = Translation3f(tra)*Quaternionf(rot);
    PointPairFeatures::printPose(P,"P_original:");
    MatrixXf s=PointCloudManipulation::projectPointsAndNormals(P, m);

    //now we simulate that there is a ppf correspondence between first row of sSmall and sSmallProjected

    //PPF:planarRotationAngle
    RowVectorXf p1(6),p2(6);
    p1=m.row(0);
    p2=m.row(1); //model
    PPF ppfModel = PPF::makePPF(p1,p2,0,1);  //also gets alpha-> planar rot angle to local coords

    RowVectorXf q1(6),q2(6);
    q1=s.row(0);
    q2=s.row(1); //scene
    PPF ppfScene = PPF::makePPF(q1,q2,0,1);  //also gets alpha-> planar rot angle to local coords


    double alpha = PointPairFeatures::getAngleDiffMod2Pi(ppfModel.alpha, ppfScene.alpha);

    Isometry3f Pest = PointPairFeatures::alignSceneToModel(q1,p1,alpha);

    PointPairFeatures::printPose(Pest,"P_est:");

    PointPairFeatures::err(P,Pest);




}

