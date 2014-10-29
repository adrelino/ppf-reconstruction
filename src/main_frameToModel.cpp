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


    MatrixXd m=LoadingSaving::loadXYZ("bunny/model.xyz");
    MatrixXd mSmall=PointCloudManipulation::downSample(m,false);
    Translation3d traCentroid=PointCloudManipulation::getTranslationToCentroid(mSmall);
    mSmall=PointCloudManipulation::projectPointsAndNormals(Projective3d(traCentroid),mSmall);

    Visualize* inst = Visualize::getInstance();
    inst->model=mSmall; //PointCloudManipulation::projectPointsAndNormals(Projective3d(traCentroid),m);
    Visualize::start();

    GlobalModelDescription model = PointPairFeatures::buildGlobalModelDescription(mSmall);

    for(int i=0; i<=36; i+=2){
        stringstream ss,ss1,ss2;
        ss<<"bunny/depth-cloud/cloudXYZ_"<<i<<".xyz";
        MatrixXd cloud=LoadingSaving::loadXYZ(ss.str());

        ss1<<"bunny/depth-poses/poses_"<<i<<".txt";
        Projective3d P(LoadingSaving::loadProjectionMatrix(ss1.str()));

        ss2.flush();
        ss2<<"P:"<<i<<endl;

        PointPairFeatures::printPose(P,ss2.str());

        cloud=PointCloudManipulation::projectPointsAndNormals(P, cloud);

        MatrixXd sSmall=PointCloudManipulation::downSample(cloud,false);
        inst->scene=sSmall;
        Visualize::update();

        Projective3d Pg = PointPairFeatures::getTransformationBetweenPointClouds(mSmall,sSmall,model);

        MatrixXd cloud2=PointCloudManipulation::projectPointsAndNormals(Pg.inverse(), sSmall);

        inst->modelT=cloud2;

        inst->ms.push_back(make_pair(cloud2,Colormap::Jet(i/36.0)));
        Visualize::update();


        //Visualize::waitKey('g');

    }

     Visualize::waitKeyQuit();




}
