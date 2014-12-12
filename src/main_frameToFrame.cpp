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
    PointCloud previousScene;

    vector<Isometry3f> trajectoryGroundTruth;
    vector<Isometry3f> trajectoryEst;

//    PointCloud test;
//    test.pts.push_back(Vector3f(0,0,0));
//    test.pts_color.push_back(Vector3f(1,1,1));
//    test.nor.push_back(Vector3f(1,1,1));

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
            //test=PointCloudManipulation::projectPointsAndNormals(P_est,test);
        }else{
            //Isometry3f P_interFrame = PointPairFeatures::getTransformationBetweenPointClouds(previousScene,sSmall);

            //Isometry3f P_interFrame_groundTruth = P*trajectoryGroundTruth[i-1].inverse();
            //cout<<"inter frame:"<<endl;
            //err(P_interFrame,P_interFrame_groundTruth,true);

            //test=PointCloudManipulation::projectPointsAndNormals(P_interFrame,test);
            //Isometry3f P_est2 = (P_interFrame * trajectoryGroundTruth[i-1]);

            P_est = PointPairFeatures::getTransformationBetweenPointClouds(sSmall,previousFrame);
            //cout<<"pests pest2"<<endl;
            //err(P_est,P_est2,true);

        }

        trajectoryGroundTruth.push_back(P);
        trajectoryEst.push_back(P_est);

        //Visualize::addCloud(std::make_pair(test,Vector3f(0,0,0)));

        previousScene=sSmall;


        previousFrame=PointCloudManipulation::projectPointsAndNormals(P_est,sSmall);

        previousFrame.pts_color.push_back(Colormap::Jet(i/36.0f));


        cout<<"Error between PPF pose and groundTruth:"<<endl;
        err(P,P_est,true);

        Visualize::setModelTransformed(previousFrame);

        Visualize::addCameraPose(P_est);
        Visualize::addCameraPoseGroundTruth(P);

        Visualize::spin();

        Isometry3f P_Iterative_ICP = P; //initialize with ppf coarse alignment
        PointCloud cloud2;

        int j = 0;
        for (; j < 5; ++j) {  //5 ICP steps

            cloud2=PointCloudManipulation::projectPointsAndNormals(P_Iterative_ICP/*.inverse()*/, sSmall);
            //all.append(cloud2);
            vector<Vector3f> src,dst;

            PointCloudManipulation::getClosesPoints(cloud2,mSmall,src,dst,0.03f);

            Visualize::setModelTransformed(cloud2);
            Visualize::setLines(src,dst);

            cout<<"Iteration "<<i<<endl;
            cout<<"# Scene to Model Correspondences: "<<src.size()<<"=="<<dst.size()<<endl;


            cout<<"ICP "<<endl;

            //ICP::getTransformationBetweenPointClouds(mSmall,cloud2);
            Isometry3f P_incemental = ICP::computeStep(src,dst,false);
            printPose(P_incemental, "P incremental ICP");
            if(PointPairFeatures::isPoseCloseToIdentity(P_incemental,0.000001)){
                break;
            }
            P_Iterative_ICP = P_incemental * P_Iterative_ICP;

            Visualize::setLastCameraPose(P_Iterative_ICP);
            //printPose(P_Iterative_ICP, "Piterative ICP");

            previousFrame=PointCloudManipulation::projectPointsAndNormals(P_Iterative_ICP,sSmall);



            Visualize::spin();


        }

        Visualize::addCloud(std::make_pair(previousFrame,Vector3f(0,1,0)));




    }




}
