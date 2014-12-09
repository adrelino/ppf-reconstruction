//
//  main.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 27.06.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//
#include "Visualize.h"
#include "LoadingSaving.h"
#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
#include <iostream>
#include "Constants.h"

using namespace std;

int main(int argc, char * argv[])
{
    PointCloud mSmall=PointCloudManipulation::downSample(LoadingSaving::loadPointCloud("bunny/bunny_smoothNormals.xyz"),ddist);
    PointCloudManipulation::reestimateNormals(mSmall,ddist);
    Visualize::setModel(mSmall);

    PointCloud sSmall=PointCloudManipulation::downSample(LoadingSaving::loadPointCloud("bunny/bunny_smoothNormals.xyz",300),ddist);
    PointCloudManipulation::reestimateNormals(sSmall,ddist);

    Quaternionf q=Quaternionf::Identity();
    q = q * AngleAxisf(deg2rad(-22), Vector3f::UnitX());
    q = q * AngleAxisf(deg2rad(123),Vector3f::UnitY());
    q = q * AngleAxisf(deg2rad(-379),Vector3f::UnitZ());

    Vector4f rot(q.x(),q.y(),q.z(),q.w());
    //Vector4f rot(.2,.2,.2,.4);
    //Vector3f tra(0,0,0);//
    Vector3f tra(0.05,0.2,-0.07);//,0.5,0.01);

    Isometry3f P = Translation3f(tra)*Quaternionf(rot);
    sSmall=PointCloudManipulation::projectPointsAndNormals(P, sSmall);
    Visualize::setScene(sSmall);



    Isometry3f Pest=PointPairFeatures::getTransformationBetweenPointClouds(mSmall,sSmall);

    Visualize::addCameraPose(Pest);

    cout<<"PPF coarse P"<<endl;
    err(P,Pest);


    Isometry3f P_Iterative_ICP = Pest; //initialize with ppf coarse alignment
    PointCloud modelPoseEst;

    int i;

    for (i=0; i < 100; ++i) {  //5 ICP steps
        modelPoseEst=PointCloudManipulation::projectPointsAndNormals(P_Iterative_ICP, mSmall);

        vector<Vector3f> src,dst;

        PointCloudManipulation::getClosesPoints(modelPoseEst,sSmall,src,dst,0.03f);

        Visualize::setModelTransformed(modelPoseEst);
        Visualize::setLines(src,dst);

        //cout<<"Iteration "<<i<<endl;
        //cout<<"# Scene to Model Correspondences: "<<dst.size()<<endl;

        Visualize::spin();

        Isometry3f P_incemental = ICP::computeStep(src,dst,false);

        if(PointPairFeatures::isPoseCloseToIdentity(P_incemental,0.00001)){
            break;
        }

        P_Iterative_ICP = P_incemental * P_Iterative_ICP;

        //cout<<"ICP Step "<<i<<endl;
        err(P,P_Iterative_ICP);

    }

    cout<<"ICP converged after #Steps= "<<i<<endl;

    Visualize::spinLast();

}
