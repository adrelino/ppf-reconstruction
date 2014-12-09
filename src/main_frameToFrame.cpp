//
//  main_frameToFrame.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 08.12.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//
#include "Visualize.h"
#include "LoadingSaving.h"
#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
#include <iostream>
#include <string>
#include "Constants.h"

using namespace std;

int main(int argc, char * argv[])
{
    PointCloud m=LoadingSaving::loadPointCloud("bunny/bunny_smoothNormals.xyz");
    PointCloud mSmall=PointCloudManipulation::downSample(m,ddist);
    PointCloudManipulation::reestimateNormals(mSmall,ddist);

    Visualize::setModel(mSmall);

    Visualize::spin(1);

   // PointCloud all = PointCloud();

    PointCloud previousFrame;

    vector<Isometry3f> trajectoryGroundTruth;
    vector<Isometry3f> trajectoryEst;

    for(int i=0; i<=36; i++){
        stringstream ss,ss1,ss2;

        ss<<"bunny/depth-cloud/cloudXYZ_"<<i<<".xyz";

        PointCloud cloud=LoadingSaving::loadPointCloud(ss.str());
        PointCloud sSmall=PointCloudManipulation::downSample(cloud,ddist);
        PointCloudManipulation::reestimateNormals(sSmall,ddist);

        //groundTruth
        ss1<<"bunny/depth-poses/cloudToPLY-coarse_"<<i<<".txt";
        Isometry3f P(LoadingSaving::loadMatrix4f(ss1.str()));
        Isometry3f P_est;

        //get inter frame motion
        if(i==0){
            P_est=P;
        }else{
            Isometry3f P_interFrame = PointPairFeatures::getTransformationBetweenPointClouds(previousFrame,sSmall);
            Isometry3f P_interFrame_groundTruth = trajectoryGroundTruth[i-1] * P.inverse();
            err(P_interFrame,P_interFrame_groundTruth);

            P_est = (P_interFrame * trajectoryGroundTruth[i-1]);
        }

        trajectoryGroundTruth.push_back(P);
        trajectoryEst.push_back(P);


        previousFrame=PointCloudManipulation::projectPointsAndNormals(P_est,sSmall);
        err(P,P_est);

        Visualize::setModelTransformed(previousFrame);

        Visualize::addCameraPose(P_est);
        Visualize::addCameraPoseGroundTruth(P);

        Visualize::spin();
    }




}
