//
//  mainSequence.cpp
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

    Visualize* inst = Visualize::getInstance();
    inst->current_object=1;

    Visualize::start();


    for(int i=0; i<=36; i++){

        stringstream ss,ss1,ss2;
        ss<<"bunny/depth-cloud/cloudXYZ_"<<i<<".xyz";
        MatrixXd cloud=LoadingSaving::loadXYZ(ss.str());

        ss1<<"bunny/depth-poses/poses_"<<i<<".txt";
        Projective3d P(LoadingSaving::loadProjectionMatrix(ss1.str()));

        ss2.flush();
        ss2<<"P:"<<i<<endl;

        PointPairFeatures::printPose(P,ss2.str());

        cloud=PointCloudManipulation::projectPointsAndNormals(P, cloud);

        inst->ms.push_back(make_pair(cloud,RowVector3f(i/36.0,0,1-i/36.0)));
        Visualize::update();

    }

    Visualize::waitKeyQuit();

    //Visualize::visualize();


}
