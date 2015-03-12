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

//#include "G2OWrapper.h"

#include "ApproachComponents.h"

using namespace std;

CPUTimer timer = CPUTimer();

void easy(vector< std::shared_ptr<PointCloud> >& frames,
          int nGraphUpdates, bool useICPdist,
          float tra_thresh, float rot_thresh,
          float knn, float cutoff, int startFrame,
          bool stacked){

    float allFixed;
    int n = frames.size();



    for(int l=0; l<30; l++){

        for(auto it : frames){
            it->fixed=false;
        }


    frames[0]->fixed=true;


    for(int k=0; k<nGraphUpdates; k++){ //Pose Graph updates

        for(int i=startFrame; i<n; i++){
            if(!useICPdist){
                frames[i]->computePoseNeighbours(&frames,i,tra_thresh,rot_thresh);
            }else{
                //frames[i]->computeCloudNeighbours(&frames,i,overlap,cutoff,mean_nn_thresh);
                frames[i]->computeCloudNeighboursKnn(&frames,i,knn,cutoff);
            }
        }

        cout<<"[Graph Update "<<k<<"] pose edges computed"<<endl;
        Visualize::spinToggle(2);


        bool allIcpConverged=false;

        //Global icp
        for (int j=0; j<25 && !allIcpConverged;j++) {  //ICP

            //timer.tic();

            vector<Isometry3f> P_incrementals(frames.size());

            float accumErr=0;
            float accumICPErr=0;

            allIcpConverged=true;

            for(int i=startFrame; i<n; i++){  //leave frame 0 fixed;

                if(frames[i]->neighbours.size()>0 && !frames[i]->fixed){
                    if(stacked){
                        accumICPErr += frames[i]->computeClosestPointsToNeighboursStacked(&frames,Params::getInstance()->ddist);
                    }else{
                        accumICPErr += frames[i]->computeClosestPointsToNeighbours(&frames,Params::getInstance()->ddist);
                    }

                    Isometry3f P_incemental;
//                    if(Params::getInstance()->pointToPlane){
//                       /*Isometry3f*/ P_incemental = ICP::computeStep(frames[i]->src,frames[i]->dst,frames[i]->dstNor); //point to plane

//                        //printPose(P_incemental,"normal icp");

//                        //Isometry3f
//                        //        P_incemental = g2oWrapper::computeStep(frames[i]->src,frames[i]->srcNor,frames[i]->dst,frames[i]->dstNor); //point to plane

////                        printPose(P_incemental2,"lm global icp");

//                        //cout<<"error: "<<err(P_incemental,P_incemental2,false,true)<<endl;

////                        Visualize::spin();

//                    }else{
//                        P_incemental = ICP::computeStep(frames[i]->src,frames[i]->dst,false); //point to point
//                    }

//                    if(PointPairFeatures::isPoseCloseToIdentity(P_incemental,0.001)){
//                        frames[i]->fixed=true;
//                    }else{
//                        allIcpConverged=false;
//                        //cout<<"not conv: "<<i<<endl;

//                    }

                    P_incrementals[i] = P_incemental;
                }
            }

            allFixed=true;

            for(int i=startFrame; i<frames.size(); i++){
                if(!frames[i]->fixed){
                    allFixed=false;
                    if(frames[i]->neighbours.size()>0){

                        //printPose(P_incemental, "P incremental ICP");
                        Isometry3f P_rereferenced = P_incrementals[i];
                        if(startFrame==0){
                            P_rereferenced = P_rereferenced* P_incrementals[0].inverse();
                        }

                        Isometry3f P_est = P_rereferenced * frames[i]->pose;
                        frames[i]->setPose(P_est);
                    }
                }
                accumErr+=frames[i]->getPoseError();
            }

            cout<<"[Global ] [ICP "<<j<<"] GroundTruth Error: "<<accumErr<<"\t ICP dist error:"<<accumICPErr<<" allIcpConverged: "<<allIcpConverged<<endl;//<<" time: "<<timer.tocSeconds()<<endl;

            Visualize::spinToggle(2);
        }

        if(allFixed) break;


    }
    cout<<"next fixed round"<<endl;
    }

}

int main(int argc, char * argv[])
{
    //for pose edges
    float tra_thresh = 0.35f; //0.02f * diamM; //float thresh_tra=0.02; //2cm
    float rot_thresh = 20;

    int nGraphUpdates = 250;

    getParam("tra_thresh",tra_thresh,argc,argv);
    getParam("rot_thresh",rot_thresh,argc,argv);

    float overlap=0.7f;
    float mean_nn_thresh=0.02f;

    getParam("overlap", overlap, argc, argv);
    getParam("mean_nn_thresh", mean_nn_thresh, argc, argv);

    bool useICPdist = true;
    getParam("useICPdist", useICPdist, argc, argv);
    getParam("nGraphUpdates",nGraphUpdates,argc,argv);

    bool stacked = true;
    getParam("stacked",stacked,argc,argv);

    int knn=6; //knn transl diff poses
    getParam("knn",knn,argc,argv);

    int iter=500;
    getParam("iter",iter,argc,argv);


    int startFrame=1;


    Params* inst = Params::getInstance();
    inst->getParams(argc,argv);

    float cutoff=inst->ddist*2;//0.01f;  //d_max
    getParam("cutoff", cutoff, argc, argv);

    float huberWidth=inst->ddist*0.5f;
    getParam("huberWidth", huberWidth, argc, argv);



    vector<string> images = LoadingSaving::getAllImagesFromFolder(Params::getInstance()->dir,"depth");
    vector<string> imagesMasks = LoadingSaving::getAllImagesFromFolder(Params::getDir(),"*mask");

    vector<Isometry3f> posesGroundTruth = LoadingSaving::loadPoses(Params::getInstance()->dir,"pose");
    vector<Isometry3f> posesEst = LoadingSaving::loadPosesFromDir(Params::getInstance()->dir,Params::getInstance()->est_poses_prefix); //estimated poses
    vector<string> posesEstFileNames = LoadingSaving::getAllTextFilesFromFolder(Params::getInstance()->dir,Params::getInstance()->est_poses_prefix);

    if(posesEstFileNames.size()==0){
        cerr<<"you need to run frameToFrame first to get a first pose estimate"<<endl;
        exit(1);
    }

    vector<string> intrinsics = LoadingSaving::getAllTextFilesFromFolder(Params::getInstance()->dir,"Intrinsic");
    Matrix3f K = LoadingSaving::loadMatrix3f(intrinsics[0]);

    vector< std::shared_ptr<PointCloud> > frames;

    Visualize::setClouds(&frames);

    for(int i_frame=0; i_frame<posesEstFileNames.size(); i_frame++){
        string estimate = posesEstFileNames[i_frame];
        string number = estimate.substr(estimate.length()-Params::getInstance()->est_poses_suffix.length()-6,6);

        int i = atoi(number.c_str());
        cout<<"est "<<estimate<<" number "<<i<<endl;

        string maskname = ""; //no mask
        if(i<imagesMasks.size()){
            maskname=imagesMasks[i];
        }

        shared_ptr<PointCloud> currentFrame(new PointCloud(images[i],K,maskname));

        currentFrame->downsample(Params::getInstance()->ddist);

        //Transformation groundTruth
        Isometry3f P = posesGroundTruth[i];
        currentFrame->setPoseGroundTruth(P);
        currentFrame->imgSequenceIdx=i;

        //Transformation estimate
        Isometry3f P_est = posesEst[i_frame];
        currentFrame->setPose(P_est);

        frames.push_back(currentFrame);

        if(i_frame==0){
            Visualize::getInstance()->setOffset((-1)*(currentFrame->pose*currentFrame->centerOfMass));
        }

    }

    float originalErrorTra=PointCloudManipulation::registrationErrorTra(frames);
    float originalErrorRot=PointCloudManipulation::registrationErrorRot(frames);

    Visualize::setCallbackForKey('k',[&]() -> void{
       cout<<"tra: "<<originalErrorTra<<" rot: "<<originalErrorRot<<endl;
       cout<<"tra: "<<PointCloudManipulation::registrationErrorTra(frames)<<" rot: "<<PointCloudManipulation::registrationErrorRot(frames)<<endl;
     });


    Visualize::setCallbackForKey('s',[&]() -> void{
        LoadingSaving::savePosesEvalutationEstimates(frames,"multiview");
       });


    //easy(frames,nGraphUpdates, useICPdist,tra_thresh, rot_thresh,knn, cutoff, startFrame,stacked);

    ApproachComponents::g2oOptimizer(frames,cutoff,iter,knn,huberWidth);

    cout<<"Finished"<<endl;


    Visualize::spinLast();

}
