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
    int start = 0;
    bool doICP = true;


    bool showDepthMap = false;

    int step = 1;
    int nFrames = 1; //number of last frames for ppf matching

    getParam("start", start, argc, argv);
    getParam("doICP", doICP, argc, argv);

    getParam("showDepthMap", showDepthMap, argc, argv);
    getParam("step", step, argc, argv);
    getParam("nFrames", nFrames, argc, argv);

    Params* inst = Params::getInstance();
    inst->getParams(argc,argv);

    vector<string> images = LoadingSaving::getAllImagesFromFolder(Params::getDir(),"*depth");
    vector<string> imagesMasks = LoadingSaving::getAllImagesFromFolder(Params::getDir(),"*mask");
    vector<string> intrinsics = LoadingSaving::getAllTextFilesFromFolder(Params::getDir(),"Intrinsic");

    if(intrinsics.size()!=1){
        cerr<<"can't find Intrinisc.txt file"<<endl;
        exit(1);
    }
    Matrix3f K = LoadingSaving::loadMatrix3f(intrinsics[0]);

    vector<Isometry3f> posesGroundTruth = LoadingSaving::loadPoses(Params::getDir(),"pose");

    vector< std::shared_ptr<PointCloud> > frames;

    Visualize::setClouds(&frames);

    int i_frame=0;

    for(int i=start; i<images.size(); i+=step, i_frame++){

        cout<<"[Frame "<<i_frame<<" Image "<<i<<"] ";

        Visualize::setSelectedIndex(i_frame);

        string maskname = ""; //no mask
        if(i<imagesMasks.size()){
            maskname=imagesMasks[i];
        }

        shared_ptr<PointCloud> currentFrame(new PointCloud(images[i],K,maskname,showDepthMap));

        //cout<<"downsample"<<endl;
        currentFrame->downsample(inst->ddist);
        //cout<<"framePush"<<endl;
        frames.push_back(currentFrame);
        //cout<<"after framePush"<<endl;

        currentFrame->imgSequenceIdx=i;


        Isometry3f P = Isometry3f::Identity();
        //Transformation groundTruth
        if(i<posesGroundTruth.size()){
            P = posesGroundTruth[i];
            currentFrame->setPoseGroundTruth(P);
        }


        //Transformation estimate
        Isometry3f P_est = Isometry3f::Identity();
        //get inter frame motion
        if(i==start){
            P_est=P;
        }else{
            currentFrame->setPose(P_est);

            int ppfMaxVotes = -1;

            int j;

            for(j=i_frame-1; j>=i_frame-nFrames && j>=0;j--){ //consider last 5 frames at most
                cout<<"ppf "<<j<<endl;
                Poses poses = PointPairFeatures::getTransformationBetweenPointClouds(*currentFrame,*frames[j]); //frames.back() for drift

//                for(int k=0; k>=0; k--){
//                   P_est = poses[k].first;
//                   trajectoryEst.back()=P_est;
//                   //currentFrame->setPose(P_est);
////                   PointCloud test = currentFrame.projected(P_est);
////                   vector<Vector3f> src, dst;
////                   float closestPointsError = PointCloudManipulation::getClosesPoints(test,frames[j],src,dst,ddist);
////                   cout<<"[Frame "<<i<<"] [PPF "<<j<<"] ["<<k<<" best Pose] groundTruth error:"<<err(P,P_est);
////                   cout<<" score: "<<poses[k].second<<"\t ICP dist error:"<<closestPointsError<<endl;
////                   Visualize::setLines(src,dst);
////                   Visualize::setSelectedIndex(i);
//                   //Visualize::spin();
//                }

                cout<<"ppf after"<<endl;

                if(poses[0].second>=ppfMaxVotes){ //ppfMinVotes){
                    ppfMaxVotes=poses[0].second;
                    stringstream ss;
                    ss<<"votes: "<<ppfMaxVotes<<endl;
                    P_est=poses[0].first;
                    currentFrame->neighboursDescr=ss.str();
                    currentFrame->neighbours.clear();
                    currentFrame->neighbours.push_back(make_pair(j,ppfMaxVotes));
                    //currentFrame->setPose(P_est);
                }/*else{
                    cout<<"score too low, click q for next try"<<endl;
                    Visualize::spin();
                }*/


            }

            cout<<"[Frame "<<i_frame<<"] [PPF to Frame "<<currentFrame->neighbours.back().first<<"] \tmaxVotes: "<<ppfMaxVotes<<endl;
        }

        currentFrame->setPose(P_est);

        if(i==start){

        }else{

            if(doICP){
                int j;
                float icpInlierError;
                for (j=0; j < 50;j++) {  //ICP
                    icpInlierError = currentFrame->computeClosestPointsToNeighbours(&frames,inst->ddist);

                    Isometry3f P_incemental;
                    if(inst->pointToPlane){
                        P_incemental = ICP::computeStep(currentFrame->src,currentFrame->dst,currentFrame->dstNor);//*src,*dst,*nor); //point to plane
                    }else{
                        P_incemental = ICP::computeStep(currentFrame->src,currentFrame->dst,false); //point to point
                    }

                    if(PointPairFeatures::isPoseCloseToIdentity(P_incemental,0.00001)){
                        break;
                    }
                    P_est = P_incemental * P_est;

                    currentFrame->setPose(P_est);
                }
            cout<<"[Frame "<<i_frame<<"] [ICP "<<j<<"] GroundTruth error:"<<err(P,P_est)<<"\t ICP dist error:"<<icpInlierError<<endl;
            Visualize::spinToggle(2);

            } //end doICP



        }//end else if i>0

        Visualize::spinToggle(2);
    }

    Visualize::spinLast();

}
