//
//  main.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 27.06.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//
#include "Visualize.h"
#include "LoadingSaving.h"
#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
#include <iostream>

using namespace std;

int main(int argc, char * argv[])
{
    
    
    //MatrixXd m = MatrixXd::Random(200,6);
    
    MatrixXd m=LoadingSaving::loadXYZ("bunny/model.xyz");//.block(0, 0, 1000, 6);
    MatrixXd mSmall=PointCloudManipulation::downSample(m,false);
    
    //double diamM=PointCloudManipulation::getPointCloudDiameter(mSmall);

    
    //LoadingSaving::saveXYZ("bunny/model_downSampled.xyz", mSmall);
    //MatrixXd mSmallVoxelCenter=PointPairFeatures::downSample(m,true);
    
    //Visualize::getInstance()->ms.push_back({m,RowVector3f(1,0,0)});
    //Visualize::getInstance()->ms.push_back({mSmall,RowVector3f(1,0.5,0)});
    //Visualize::getInstance()->ms.push_back({mSmallVoxelCenter,RowVector3f(1,0.9,0)});

    
    MatrixXd s=LoadingSaving::loadXYZ("bunny/scene.xyz");
    MatrixXd sSmall=PointCloudManipulation::downSample(s,false);
    //LoadingSaving::saveXYZ("bunny/scene_downSampled.xyz", sSmall);
    
    Projective3d P(Translation3d(0.3, 0, 0));//*AngleAxisd(M_PI_2, Vector3d(1,1,1));// * Scaling(s);

    sSmall=PointCloudManipulation::projectPointsAndNormals(P, sSmall);

    Visualize* inst = Visualize::getInstance();

    inst->model=mSmall;

    MatrixXd mSmallCentroidAtOrigin=PointCloudManipulation::translateCentroidToOrigin(mSmall);

    inst->scene=mSmallCentroidAtOrigin;
    //inst->scene=sSmall;

    cout<<"start vis Thread"<<endl;

    Visualize::start();

    Visualize::waitKeyQuit();



    cout<<"after start vis Thread"<<endl;


    PointPairFeatures* ppfs=new PointPairFeatures();


    
    //Visualize::getInstance()->ms.push_back({sSmall,RowVector3f(0.5,1,0)});
    
    GlobalModelDescription map =  ppfs->buildGlobalModelDescription(mSmall);
    
    Matches matches = ppfs->matchSceneAgainstModel(sSmall, map);
    
    vector<MatrixXi> accVec=ppfs->voting(matches);
    
    cout<<accVec[0]<<endl;
    
    Poses Pests = ppfs->computePoses(accVec, mSmall, sSmall);
    Projective3d Pest=ppfs->clusterPoses(Pests);

    cout<<"groundtruth:"<<endl;
    cout<<P.matrix()<<endl;

    cout<<"estimated:"<<endl;
    cout<<Pest.matrix()<<endl;


    
    
    MatrixXd modelPoseEst=PointCloudManipulation::projectPointsAndNormals(Pest, m);


    //printMap(map);
    //KeyBucketPairList best10=PointPairFeatures::print10(map);
    
    //Visualize::getInstance()->b=best10;

    //Visualize::waitKey('g');

    cout<<"after waitKey"<<endl;
    

    inst->modelT=modelPoseEst;
    inst->matches=matches;

    Visualize::waitKeyQuit();

}
