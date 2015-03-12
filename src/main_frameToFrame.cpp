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

#include "OpenCVHelpers.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iomanip>

using namespace std;

int main(int argc, char * argv[])
{
    int start = 0;
    bool doICP = true;
    bool showDepthMap = false;
    int step = 1;
    int nFrames = 4; //number of last frames for ppf matching

    getParam("start", start, argc, argv);
    getParam("doICP", doICP, argc, argv);
    getParam("showDepthMap", showDepthMap, argc, argv);
    getParam("step", step, argc, argv);
    getParam("nFrames", nFrames, argc, argv);


    Params* inst = Params::getInstance();

    float cutoff=inst->ddist*2;//0.01f;
    getParam("cutoff", cutoff, argc, argv);

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

    LoadingSaving::savePosesEvalutationGroundTruth(posesGroundTruth);

    vector< std::shared_ptr<PointCloud> > frames;

    Visualize::setClouds(&frames);


    Visualize::setCallbackForKey('s',[&]() -> void{
        int i;
        for (i = 0; i < frames.size(); ++i) {
            std::stringstream ss;
            PointCloud& cloud = *frames[i];
            ss<<Params::getInstance()->dir<<"/"<<Params::getInstance()->est_poses_prefix<<std::setfill('0')<<std::setw(6)<<cloud.imgSequenceIdx<<Params::getInstance()->est_poses_suffix;
            LoadingSaving::saveMatrix4f(ss.str(),cloud.pose.matrix());
        }
        cout<<"Saved "<<i<<" estimated poses in dir: "<<Params::getInstance()->dir<<endl;

        LoadingSaving::savePosesEvalutationEstimates(frames,"pairwise");
       });



//    Visualize::setCallbackForKey('k',[&]() -> void{
////       cout<<"tra: "<<originalErrorTra<<" rot: "<<originalErrorRot<<endl;
//       cout<<"tra: "<<PointCloudManipulation::registrationErrorTra(frames)<<" rot: "<<PointCloudManipulation::registrationErrorRot(frames)<<endl;
//     });

    int i_frame=0;

    for(int i=start; i<images.size(); i+=step, i_frame++){

        cout<<"[Frame "<<i_frame<<" Image "<<i<<"] ";

        Visualize::setSelectedIndex(i_frame);
        if(i_frame>0) Visualize::getInstance()->ingoingEdgeFrame=i_frame-1;

        string maskname = ""; //no mask
        if(i<imagesMasks.size()){
            maskname=imagesMasks[i];
        }

        shared_ptr<PointCloud> currentFrame(new PointCloud(images[i],K,maskname,showDepthMap));

        Isometry3f P = Isometry3f::Identity();
        //Transformation groundTruth
        if(i<posesGroundTruth.size()){
            P = posesGroundTruth[i];
            currentFrame->setPoseGroundTruth(P);
        }

        //Transformation estimate
        Isometry3f P_est = Isometry3f::Identity();
        //get inter frame motion
        if(i==start) P_est=P;

        currentFrame->setPose(P_est);
        currentFrame->imgSequenceIdx=i;

        currentFrame->downsample(inst->ddist);
        frames.push_back(currentFrame);

        if(i==start){
            Visualize::getInstance()->setOffset((-1)*(currentFrame->pose*currentFrame->centerOfMass));
        }


        //Visualize::spin();



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
                   // stringstream ss;
                   // ss<<"votes: "<<ppfMaxVotes<<endl;
                    P_est=poses[0].first;
                    currentFrame->neighboursDescr="votes";//ss.str();
                    currentFrame->neighbours.clear();
                    currentFrame->neighbours.push_back({j,static_cast<float>(ppfMaxVotes)});
                    //currentFrame->setPose(P_est);
                }/*else{
                    cout<<"score too low, click q for next try"<<endl;
                    Visualize::spin();
                }*/


            }

            cout<<"[Frame "<<i_frame<<"] [PPF to Frame "<<currentFrame->neighbours.back().neighbourIdx<<"] \tmaxVotes: "<<ppfMaxVotes<<endl;
        }

        currentFrame->setPose(P_est);

        if(i==start){

        }else{

            if(doICP){
                int j;
                float icpInlierError;
                for (j=0; j < 50;j++) {  //ICP
                    icpInlierError = currentFrame->computeClosestPointsToNeighbours(&frames,cutoff);

                    //cout<<"[Frame "<<i_frame<<"] [ICP "<<j<<"] ICP dist error:"<<icpInlierError<<endl;


                    if(currentFrame->alignToFirstNeighbourWithICP(&frames,inst->pointToPlane,false)) break;
                    P_est=currentFrame->pose;
                    Visualize::spin(3);

                }
                cout<<"[Frame "<<i_frame<<"] [ICP "<<j<<"] GroundTruth error:"<<err(P,P_est)<<"\t ICP dist error:"<<icpInlierError<<endl;
            //Visualize::spinToggle(5);

            } //end doICP



        }//end else if i>0

        Visualize::spinToggle(2);
    }


    if(!doICP){
        cout<<"now start icp"<<endl;
        Visualize::spin();


        for(int i_frame=start+1;i_frame<frames.size(); i_frame++){

            cout<<"[Frame "<<i_frame;

            Visualize::setSelectedIndex(i_frame);
            if(i_frame>0) Visualize::getInstance()->ingoingEdgeFrame=i_frame-1;

            shared_ptr<PointCloud> currentFrame=frames[i_frame];

            int j;
            float icpInlierError;
            for (j=0; j < 50;j++) {  //ICP
                icpInlierError = currentFrame->computeClosestPointsToNeighboursRelative(&frames,cutoff);

                //cout<<"[Frame "<<i_frame<<"] [ICP "<<j<<"] ICP dist error:"<<icpInlierError<<endl;


                if(currentFrame->alignToFirstNeighbourWithICP(&frames,inst->pointToPlane,true)) break;

                Visualize::spin(3);

            }
            cout<<"[Frame "<<i_frame<<"] [ICP "<<j<<"] GroundTruth error:"<<err(currentFrame->poseGroundTruth,currentFrame->pose)<<"\t ICP dist error:"<<icpInlierError<<endl;

        }
    }


    Visualize::spinLast();

}
