//
//  main_frameToModel.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 29.10.14.
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

    TrainedModel trainedModel = PointPairFeatures::trainModel(mSmall);

    for(int i=0; i<=36; i++){
        stringstream ss,ss1,ss2;

        ss<<"bunny/depth-cloud/cloudXYZ_"<<i<<".xyz";

        PointCloud cloud=LoadingSaving::loadPointCloud(ss.str());

        ss1<<"bunny/depth-poses/cloudToPLY-coarse_"<<i<<".txt";
        Isometry3f P_gold(LoadingSaving::loadMatrix4f(ss1.str()));


        PointCloud sSmall=PointCloudManipulation::downSample(cloud,ddist);
        PointCloudManipulation::reestimateNormals(sSmall,ddist);

        Visualize::setScene(sSmall);

        Isometry3f P_est = PointPairFeatures::getTransformationBetweenPointClouds(trainedModel,sSmall).inverse();

        err(P_gold,P_est);


        Isometry3f P_Iterative_ICP = P_est; //initialize with ppf coarse alignment
        PointCloud cloud2;


        Visualize::addCameraPoseGroundTruth(P_gold);
        Visualize::addCameraPose(P_Iterative_ICP);

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

            err(P_gold,P_Iterative_ICP);



            Visualize::spin();


        }

        //cout<<"finished ICP after "<<j<<" iterations, press q to continue with next frame"<<endl;

        //inst->ms.push_back(make_pair(cloud2,Colormap::Jet(i/36.0)));
        cout<<"cloud "<<i<<"/ #clouds: "<<i<<" #pts="<<cloud.rows()<<endl;

        Visualize::spin();
    }
//    cout<<"all #pts="<<all.rows()<<endl;
//    //inst->scene=all;
//    PointCloud sSmall=PointCloudManipulation::downSample(all,ddist);
//    PointCloudManipulation::reestimateNormals(sSmall,ddist);
//    cout<<"all downsamplet #pts="<<sSmall.rows()<<endl;

//    inst->scene=sSmall;



//    Visualize::spin();


//    //TrainedModel trainedModel = PointPairFeatures::trainModel(mSmall);

//    //Isometry3f P = PointPairFeatures::getTransformationBetweenPointClouds(sSmall,mSmall);
//    //P=P.inverse();
//    //inst->modelT = PointCloudManipulation::projectPointsAndNormals(P,mSmall);

//    cout<<"PPF P"<<endl;
//    Visualize::spin();

//    //Isometry3f Pref = ICP::getTransformationBetweenPointClouds(sSmall,inst->modelT,50,0.00001f);

//    //inst->modelT = PointCloudManipulation::projectPointsAndNormals(Pref,inst->modelT);

//    PointPairFeatures::printPose(Pref, "P ref");


//    Visualize::spin();

}
