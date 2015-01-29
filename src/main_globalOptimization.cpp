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

#include "CPUTimer.h"

using namespace std;

CPUTimer timer = CPUTimer();

int main(int argc, char * argv[])
{
    std::string dir = "samples/Bunny_RealData/";
    //std::string dir = "samples/Bunny_Sphere";

    bool pointToPlane = true; //otherwise pointToPoint
    bool useFlann = true;
    bool downsample = true;

    //for pose edges
    float tra_thresh = 0.35f; //0.02f * diamM; //float thresh_tra=0.02; //2cm
    float rot_thresh = 20;

    int nGraphUpdates = 250;



    getParam("dir", dir, argc, argv);
    getParam("useFlann",useFlann,argc,argv);
    getParam("pointToPlane",pointToPlane,argc,argv);
    getParam("downsample",downsample,argc,argv);

    getParam("tra_thresh",tra_thresh,argc,argv);
    getParam("rot_thresh",rot_thresh,argc,argv);
    getParam("nGraphUpdates",nGraphUpdates,argc,argv);


    getParams(argc,argv);

    vector<string> images = LoadingSaving::getAllImagesFromFolder(dir,"depth");
    vector<Isometry3f> posesGroundTruth = LoadingSaving::loadPosesFromDir(dir,"poses");
    vector<Isometry3f> posesEst = LoadingSaving::loadPosesFromDir(dir,"est_poses_point"); //estimated poses

    if(posesEst.size()==0){
        cerr<<"you need to run frameToFrame first to get a first pose estimate"<<endl;
        exit(1);
    }

    vector<string> intrinsics = LoadingSaving::getAllTextFilesFromFolder(dir,"Intrinsic");
    Matrix3f K = LoadingSaving::loadMatrix3f(intrinsics[0]);

    vector< std::shared_ptr<PointCloud> > frames;

    Visualize::setClouds(&frames);

    for(int i=0; i<images.size(); i++){
        shared_ptr<PointCloud> currentFrame(new PointCloud(images[i],K));
        if(downsample){
        currentFrame->downsample(ddist);
        }

        //Transformation groundTruth
        Isometry3f P = posesGroundTruth[i];
        currentFrame->setPoseGroundTruth(P);

        //Transformation estimate
        Isometry3f P_est = posesEst[i];
        currentFrame->setPose(P_est);

        frames.push_back(currentFrame);
    }

    int n = frames.size();


    for(int k=0; k<nGraphUpdates; k++){ //Pose Graph updates

        for(int i=0; i<n; i++){
            frames[i]->computePoseNeighbours(&frames,i,tra_thresh,rot_thresh);
        }

     cout<<"pose edges computed"<<endl;
     Visualize::spinToggle(2);



    //if(i>0) currentFrame->neighbours.push_back(i-1);

    //Global icp
    for (int j=0; j < 1000/nGraphUpdates;j++) {  //ICP

        timer.tic();

        vector<Isometry3f> P_incrementals(frames.size());

        float accumErr=0;
        float accumICPErr=0;



        //int startHere = start+1;

        for(int i=0; i<n; i++){  //leave frame 0 fixed;

            if(frames[i]->neighbours.size()>0){

    //            shared_ptr<vector<Vector3f>> src(new vector<Vector3f>());
    //            shared_ptr<vector<Vector3f>> dst(new vector<Vector3f>());
    //            shared_ptr<vector<Vector3f>> nor(new vector<Vector3f>());
                vector<Vector3f> src1,dst1,nor1;
                vector<Vector3f>* src=&src1;
                vector<Vector3f>* dst=&dst1;
                vector<Vector3f>* nor=&nor1;

                int k = mod(i+1,n);

                //vector<PointCloud> neighbours = {frames[k],frames[k2]};
                //PointCloudManipulation::getClosesPoints(frames[i],neighbours,src,dst,ddist);


                accumICPErr += frames[i]->computeClosestPointsToNeighbours(&frames,*src,*dst,ddist,useFlann,*nor);

                //accumICPErr += PointCloudManipulation::getClosesPoints(*frames[i],*frames[k],*src,*dst,ddist,useFlann,*nor);

                Isometry3f P_incemental;
                if(pointToPlane){
                    P_incemental = ICP::computeStep(*src,*dst,*nor); //point to plane
                }else{
                    P_incemental = ICP::computeStep(*src,*dst,false); //point to point
                }

                P_incrementals[i] = P_incemental;
            }
        }



        for(int i=0; i<frames.size(); i++){
            if(frames[i]->neighbours.size()>0){

            //printPose(P_incemental, "P incremental ICP");
            Isometry3f P_rereferenced = P_incrementals[i] * P_incrementals[0].inverse();

            Isometry3f P_est = P_rereferenced * frames[i]->pose;
            frames[i]->setPose(P_est);
            }

            accumErr+=frames[i]->getPoseError();

        }

        cout<<"[Global ] [ICP "<<j<<"] GroundTruth Error: "<<accumErr<<"\t ICP dist error:"<<accumICPErr<<" time: "<<timer.tocSeconds()<<endl;


        Visualize::spinToggle(2);

    }

    }

    Visualize::spinLast();

}
