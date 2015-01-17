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
    //objective is the trajectory == frames in same coordinate system
    vector<PointCloud> frames;
    vector<Isometry3f> trajectoryEst;

    vector<Isometry3f> trajectoryGroundTruth;


    std::string dir = "bunny/Bunny_RealData";
    //std::string dir = "bunny/Bunny_Sphere";

    getParam("dir", dir, argc, argv);

    vector<string> images = LoadingSaving::getAllImagesFromFolder(dir,"depth");
    vector<string> intrinsics = LoadingSaving::getAllTextFilesFromFolder(dir,"Intrinsic");
    vector<string> poses = LoadingSaving::getAllTextFilesFromFolder(dir,"poses"); //ground truth poses

    Matrix3f K = LoadingSaving::loadMatrix3f(intrinsics[0]);
    cout<<"intrinsics: "<<endl<<K<<endl;

    std::cout<<"#images:"<<images.size()<<" #poses:"<<poses.size();

    for(int i=0; i<images.size(); i++){

        //{
            PointCloud C = LoadingSaving::loadPointCloudFromDepthMap(images[i],K,false); //true means show depth image

            //C.translateToCentroid();
            //Translation3f tra = PointCloudManipulation::getTranslationToCentroid(C);
            //C=PointCloudManipulation::projectPointsAndNormals(tra,C);

            PointCloud sSmall=PointCloudManipulation::downSample(C,ddist);
            //Visualize::setScene(sSmall);
            //Visualize::setModel(C);
            //Visualize::spin();
            //PointCloudManipulation::reestimateNormals(sSmall,ddist);
            frames.push_back(sSmall); //is later on updated in PPF coarse, ICP and global optimization;
        //}

        //Transformation groundTruth
        Isometry3f P(LoadingSaving::loadMatrix4f(poses[i]));
        trajectoryGroundTruth.push_back(P);
        Visualize::addCameraPoseGroundTruth(P);


        //Transformation estimate
        Isometry3f P_est;
        //get inter frame motion
        if(i==0){
            P_est=P;
        }else{
            P_est = PointPairFeatures::getTransformationBetweenPointClouds(frames[i],frames[i-1]);
        }
        trajectoryEst.push_back(P_est);
        Visualize::setModel(PointCloudManipulation::projectPointsAndNormals(P,frames[i])); //ground truth in green

        //update current frame point cloud coordinates
        frames[i]=PointCloudManipulation::projectPointsAndNormals(P_est,frames[i]);
        Visualize::addCameraPose(P_est);
        Visualize::addCloud(frames[i]);

        if(i==0) continue;

        cout<<"[Frame "<<i<<"] Error between PPF pose and groundTruth:";
        err(P,P_est);

        Visualize::setScene(frames[i-1]);
        Visualize::setModelTransformed(frames[i]);

        Visualize::spinToggle(20);



        //ICP::getTransformationBetweenPointClouds(mSmall,cloud2);


        for (int j=0; j < 50;j++) {  //ICP
            vector<Vector3f> src,dst;
            PointCloudManipulation::getClosesPoints(frames[i],frames[i-1],src,dst,ddist);
            Visualize::setLines(src,dst);
            Visualize::spin(3);


            //cout<<"Iteration "<<i<<endl;
            //cout<<"# Scene to Model Correspondences: "<<src.size()<<"=="<<dst.size()<<endl;
            //cout<<"ICP "<<endl;

            Isometry3f P_incemental = ICP::computeStep(src,dst,false);
            //printPose(P_incemental, "P incremental ICP");
            if(PointPairFeatures::isPoseCloseToIdentity(P_incemental,0.000001)){
                break;
            }
            P_est = P_incemental * P_est;

            cout<<"[Frame "<<i<<"] [ICP "<<j<<"] Error between PPF pose and groundTruth:";
            err(P,P_est);



            Visualize::setLastCameraPose(P_est);

            frames[i]=PointCloudManipulation::projectPointsAndNormals(P_incemental,frames[i]);
            Visualize::setModelTransformed(frames[i]);
            Visualize::setLastCloud(frames[i]);
        }

       // cv::Point3f col = colorMapped(i,36,cv::COLORMAP_HSV);

       // RowVector3f color(col.x,col.y,col.z);

        frames[i].pts_color.push_back(Map<RowVector3f>(Colormap::RED2.data()));
        frames[i].nor_color.push_back(Map<RowVector3f>(Colormap::RED1.data()));


        Visualize::spinToggle(20);




    }

    Visualize::spinLast();




}
