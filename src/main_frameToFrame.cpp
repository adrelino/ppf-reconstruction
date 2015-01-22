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
    std::string dir = "bunny/Bunny_RealData";
    //std::string dir = "bunny/Bunny_Sphere";

    int start = 0;
    bool doICP = true;
    bool savePoses = false;

    getParam("dir", dir, argc, argv);
    getParam("start", start, argc, argv);
    getParam("doICP", doICP, argc, argv);
    getParam("savePoses", savePoses, argc, argv);

    getParams(argc,argv);

    int ppfMinVotes = 50;
    getParam("ppfMinVotes", ppfMinVotes, argc, argv);





    //cout<<"sizeofPPF2: "<<sizeof(PPF2)<<endl;
    //cout<<"sizeofPPF: "<<sizeof(PPF)<<endl;


    vector<string> images = LoadingSaving::getAllImagesFromFolder(dir,"depth");
    vector<string> intrinsics = LoadingSaving::getAllTextFilesFromFolder(dir,"Intrinsic");
    vector<string> poses = LoadingSaving::getAllTextFilesFromFolder(dir,"poses"); //ground truth poses

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

        cout<<"[Frame "<<i<<"] ";

        PointCloud C = LoadingSaving::loadPointCloudFromDepthMap(images[i],K,false); //true means show depth image

        //C.translateToCentroid();

        frames.push_back(PointCloudManipulation::downSample(C,ddist));
        PointCloud& currentFrame=frames.back();

        //Transformation groundTruth
        Isometry3f P(LoadingSaving::loadMatrix4f(poses[i]));
        trajectoryGroundTruth.push_back(P);
        //Visualize::addCameraPoseGroundTruth(P);


        //Transformation estimate
        Isometry3f P_est;
        //get inter frame motion
        if(i==start){
            P_est=P;
            trajectoryEst.push_back(P_est);
        }else{

            trajectoryEst.push_back(Isometry3f::Identity());


            for(int j=i-1;j>=0;j--){
                Poses poses = PointPairFeatures::getTransformationBetweenPointClouds(currentFrame,frames[j]); //frames.back() for drift

                for(int k=3; k>=0; k--){
                   P_est = poses[k].first;
                   trajectoryEst.back()=P_est;
                   PointCloud test = currentFrame.projected(P_est);
                   vector<Vector3f> src, dst;
                   float closestPointsError = PointCloudManipulation::getClosesPoints(test,frames[j],src,dst,ddist);
                   cout<<"[Frame "<<i<<"] [PPF "<<j<<"] ["<<k<<" best Pose] groundTruth error:"<<err(P,P_est);
                   cout<<" score: "<<poses[k].second<<"\t ICP dist error:"<<closestPointsError<<endl;
                   Visualize::setLines(src,dst);
                   Visualize::setSelectedIndex(i);
                   Visualize::spinToggle(2);
                }

                P_est=poses[0].first;
                trajectoryEst.back()=P_est;





                PointPairFeatures::printPoses(poses);





                if(poses[0].second>=ppfMinVotes){
                    break;
                }else{
                    //Visualize::setLastCameraPose(P_est);
                    trajectoryEst.back()=P_est;

                    cout<<"score too low, click q for next try"<<endl;
                    Visualize::spin();
                }


            }
        }


        //PointCloud frameGroundTruth = PointCloudManipulation::projectPointsAndNormals(P,currentFrame);
        //framesGroundTruth.push_back(frameGroundTruth);
        //Visualize::setModel(frameGroundTruth); //ground truth in green

        //update current frame point cloud coordinates
        currentFrame.project(P_est);
        //frames.push_back(currentFrame);
        //Visualize::addCloud(currentFrame);

        if(i==start){

        }else{


        //Visualize::setScene(frames.back());
        //Visualize::setModelTransformed(currentFrame);



        if(doICP){
        //ICP::getTransformationBetweenPointClouds(mSmall,cloud2);
            for (int j=0; j < 50;j++) {  //ICP
                vector<Vector3f> src,dst;
                float icpInlierError = PointCloudManipulation::getClosesPoints(currentFrame,frames[i-1],src,dst,ddist);
                Visualize::setLines(src,dst);
                Visualize::spinToggle(2);


                //cout<<"Iteration "<<i<<endl;
                //cout<<"# Scene to Model Correspondences: "<<src.size()<<"=="<<dst.size()<<endl;
                //cout<<"ICP "<<endl;

                Isometry3f P_incemental = ICP::computeStep(src,dst,false);
                //printPose(P_incemental, "P incremental ICP");
                if(PointPairFeatures::isPoseCloseToIdentity(P_incemental,0.000001)){
                    break;
                }
                P_est = P_incemental * P_est;

                cout<<"[Frame "<<i<<"] [ICP "<<j<<"] GroundTruth error:"<<err(P,P_est)<<"\t ICP dist error:"<<icpInlierError<<endl;

                trajectoryEst.back()=P_est;



                //Visualize::setLastCameraPose(P_est);

                currentFrame.project(P_incemental);
                //frames.back()=currentFrame;


                //Visualize::setModelTransformed(currentFrame);
                //Visualize::setLastCloud(currentFrame);
            }
        } //end doICP

       // cv::Point3f col = colorMapped(i,36,cv::COLORMAP_HSV);

       // RowVector3f color(col.x,col.y,col.z);

        }//end if i>0

        currentFrame.pts_color.push_back(Map<RowVector3f>(Colormap::RED2.data()));
        currentFrame.nor_color.push_back(Map<RowVector3f>(Colormap::RED1.data()));
        frames.back()=currentFrame;

        if(savePoses){
            std::stringstream ss;
            ss<<dir<<"/est_poses_"<<i<<".txt";
            LoadingSaving::saveMatrix4f(ss.str(),P_est.matrix());
        }


        Visualize::spinToggle(20);
    }

    Visualize::spinLast();

}
