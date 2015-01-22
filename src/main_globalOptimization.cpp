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

//#include "OpenCVHelpers.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

int main(int argc, char * argv[])
{
    std::string dir = "bunny/Bunny_RealData";
    //std::string dir = "bunny/Bunny_Sphere";

    int start = 0;
    getParam("dir", dir, argc, argv);

    vector<string> images = LoadingSaving::getAllImagesFromFolder(dir,"depth");
    vector<string> intrinsics = LoadingSaving::getAllTextFilesFromFolder(dir,"Intrinsic");
    vector<string> poses = LoadingSaving::getAllTextFilesFromFolder(dir,"poses"); //ground truth poses
    vector<string> posesEst = LoadingSaving::getAllTextFilesFromFolder(dir,"est_poses"); //estimated poses

    if(posesEst.size()==0){
        cerr<<"you need to run frameToFrame first to get a first pose estimate"<<endl;
        exit(1);
    }


    Matrix3f K = LoadingSaving::loadMatrix3f(intrinsics[0]);
    //cout<<"intrinsics: "<<endl<<K<<endl;

    vector<PointCloud> frames;
    //vector<PointCloud> framesGroundTruth;

    vector<Isometry3f> trajectoryEst;
    vector<Isometry3f> trajectoryGroundTruth;


    Visualize::setClouds(frames);
    Visualize::setCameraPoses(trajectoryEst);
    Visualize::setCameraPosesGroundTruth(trajectoryGroundTruth);


    for(int i=start; i<images.size(); i++){
        //if(i%3==0) continue;
        PointCloud C = LoadingSaving::loadPointCloudFromDepthMap(images[i],K,false); //true means show depth image
        PointCloud currentFrame=PointCloudManipulation::downSample(C,ddist);

        //Transformation groundTruth
        Isometry3f P(LoadingSaving::loadMatrix4f(poses[i]));
        trajectoryGroundTruth.push_back(P);
        //Visualize::addCameraPoseGroundTruth(P);


        //Transformation estimate
        Isometry3f P_est(LoadingSaving::loadMatrix4f(posesEst[i]));
        //Visualize::addCameraPose(P_est);
        trajectoryEst.push_back(P_est);


        currentFrame.project(P_est);

        frames.push_back(currentFrame);

        //Visualize::addCloud(currentFrame);

        //Visualize::spinToggle(2);
    }

    Visualize::spin();

    int n = frames.size();


    //Global icp
    for (int j=0; j < 1000;j++) {  //ICP

        vector<Isometry3f> P_incrementals(frames.size());

        float accumErr=0;
        float accumICPErr=0;



        //int startHere = start+1;

        for(int i=start; i<n; i++){  //leave frame 0 fixed;
            vector<Vector3f> src,dst;
            //int k = i==start ? frames.size()-1 : i-1;
            //int k = i+1; if(k==frames.size()) k=0;

            int k = mod(i+1,n);

            //if(i==start+1){
            int k2 = mod(i-1,n);
            //}

            //int k3 = mod(i+2,n);

            //vector<PointCloud> neighbours = {frames[k],frames[k2]};
            //PointCloudManipulation::getClosesPoints(frames[i],neighbours,src,dst,ddist);
            accumICPErr += PointCloudManipulation::getClosesPoints(frames[i],frames[k],src,dst,ddist);

            //Visualize::setLines(src,dst);
            //Visualize::spin(3);

            //PointCloudManipulation::getClosesPoints(frames,i,src,dst,ddist);


            //cout<<"Iteration "<<i<<endl;
            //cout<<"# Scene to Model Correspondences: "<<src.size()<<"=="<<dst.size()<<endl;
            //cout<<"ICP "<<endl;

            Isometry3f P_incemental = ICP::computeStep(src,dst,false);
            P_incrementals[i] = P_incemental;
        }



        for(int i=start; i<frames.size(); i++){
            //printPose(P_incemental, "P incremental ICP");
            Isometry3f P_rereferenced = P_incrementals[i] * P_incrementals[0].inverse();

            trajectoryEst[i] = P_rereferenced * trajectoryEst[i];
            accumErr+=err(trajectoryGroundTruth[i],trajectoryEst[i] );

            frames[i].project(P_rereferenced);
        }

        cout<<"[Global ] [ICP "<<j<<"] GroundTruth Error: "<<accumErr<<"\t ICP dist error:"<<accumICPErr<<endl;


        Visualize::spinToggle(2);

    }

    cout<<"frame 0"<<err(trajectoryEst[0],trajectoryGroundTruth[0])<<endl;

    Visualize::spinLast();

}
