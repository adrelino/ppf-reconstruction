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

cv::Point3f colorMapped(int j, int max, int colorMap){
    cv::Mat depthMap(1,max,CV_8UC1);
    for (int i = 0; i <= max; ++i) {
        depthMap.at<uint8_t>(0,i)=i;
    }
    cv::Mat heatMap;
    cv::applyColorMap(depthMap, heatMap,colorMap);
    return heatMap.at<cv::Point3f>(0,j);

}

int main(int argc, char * argv[])
{
    //std::string dir = "samples/Bunny_RealData";
    std::string dir = "samples/Bunny_Sphere";
    //if left empty, they are assumed to be in the same folder as dir

    std::string intrinsic = "";
    std::string posesFile = "";

    //std::string rootdir = "samples/Final_Synthetic_Tests/";
    //intrinsic = rootdir + "Intrinsic.txt";
    //posesFile = rootdir + "synthetic_circle.txt";
    //std::string dir = rootdir + "Leopard/Leopard_Circle"; //"Cow/Cow_Abrupt";

    //std::array<std::string> objects = {"Bunny","Cow","JuiceBox","Kenny","Leopard","TeaBox","Teddy","Tram"};

    int start = 0;
    bool doICP = true;
    bool savePoses = false;
    bool useFlann = true;
    bool pointToPlane = false; //otherwise pointToPoint
    //int ppfMinVotes = 10;


    getParam("dir", dir, argc, argv);
    getParam("start", start, argc, argv);
    getParam("doICP", doICP, argc, argv);
    getParam("savePoses", savePoses, argc, argv);
    getParam("posesFile", posesFile, argc, argv);
    getParam("intrinsic", intrinsic, argc, argv);
    getParam("useFlann", useFlann,argc,argv);
    getParam("pointToPlane", pointToPlane,argc,argv);
    //getParam("ppfMinVotes", ppfMinVotes, argc, argv);

    getParams(argc,argv);


    cout<<"press q to start"<<endl;

    Visualize::spin();

    vector<string> images = LoadingSaving::getAllImagesFromFolder(dir,"depth");

    if(intrinsic.length()==0){
        intrinsic = LoadingSaving::getAllTextFilesFromFolder(dir,"Intrinsic")[0];
    }
    Matrix3f K = LoadingSaving::loadMatrix3f(intrinsic);

    vector<Isometry3f> posesGroundTruth;

    if(posesFile.length()==0){
       posesGroundTruth = LoadingSaving::loadPosesFromDir(dir);
    }else{
       posesGroundTruth = LoadingSaving::loadPosesFromFile(posesFile);
    }

    vector< std::shared_ptr<PointCloud> > frames;

    Visualize::setClouds(&frames);

    for(int i=start; i<images.size(); i++){

        cout<<"[Frame "<<i<<"] ";

        Visualize::setSelectedIndex(i);

        shared_ptr<PointCloud> currentFrame(new PointCloud(images[i],K));
        cout<<"downsample"<<endl;
        currentFrame->downsample(ddist);
        cout<<"framePush"<<endl;
        frames.push_back(currentFrame);
        cout<<"after framePush"<<endl;


        //Transformation groundTruth
        Isometry3f& P = posesGroundTruth[i];
        currentFrame->setPoseGroundTruth(P);

        //Transformation estimate
        Isometry3f P_est = Isometry3f::Identity();
        //get inter frame motion
        if(i==start){
            P_est=P;
        }else{
            currentFrame->setPose(P_est);

            int ppfMaxVotes = -1;

            int j;

            for(j=i-1; j>=i-6 && j>=0;j--){ //consider last 5 frames at most
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

                if(poses[0].second>=ppfMaxVotes){ //ppfMinVotes){
                    ppfMaxVotes=poses[0].second;
                    P_est=poses[0].first;
                    currentFrame->neighbours.clear();
                    currentFrame->neighbours.push_back(j);
                    //currentFrame->setPose(P_est);
                }/*else{
                    cout<<"score too low, click q for next try"<<endl;
                    Visualize::spin();
                }*/


            }

            cout<<"maxVotes: "<<ppfMaxVotes<<" between "<<i<<" and "<<currentFrame->neighbours.back()<<endl;
        }

        currentFrame->setPose(P_est);

        if(i==start){

        }else{

            if(doICP){
                for (int j=0; j < 50;j++) {  //ICP
                    shared_ptr<vector<Vector3f>> src(new vector<Vector3f>());
                    shared_ptr<vector<Vector3f>> dst(new vector<Vector3f>());
                    shared_ptr<vector<Vector3f>> nor(new vector<Vector3f>());


                    float icpInlierError = PointCloudManipulation::getClosesPoints(*currentFrame,*frames[i-1],*src,*dst,ddist,useFlann,*nor);

                    Visualize::setLines(src,dst);
                    Visualize::spinToggle(2);

                    Isometry3f P_incemental;
                    if(pointToPlane){
                        P_incemental = ICP::computeStep(*src,*dst,*nor); //point to plane
                    }else{
                        P_incemental = ICP::computeStep(*src,*dst,false); //point to point
                    }

                    if(PointPairFeatures::isPoseCloseToIdentity(P_incemental,0.000001)){
                        break;
                    }
                    P_est = P_incemental * P_est;

                    cout<<"[Frame "<<i<<"] [ICP "<<j<<"] GroundTruth error:"<<err(P,P_est)<<"\t ICP dist error:"<<icpInlierError<<endl;

                    currentFrame->setPose(P_est);

                }
            } //end doICP

        }//end else if i>0

        if(savePoses){
            std::stringstream ss;
            ss<<dir<<"/est_poses_pointPlane_"<<i<<".txt";
            LoadingSaving::saveMatrix4f(ss.str(),P_est.matrix());
        }

        Visualize::spinToggle(2);
    }

    Visualize::spinLast();

}
