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
#include "Constants.h"

using namespace std;

int main(int argc, char * argv[])
{

    Visualize* inst = Visualize::getInstance();

    MatrixXd m=LoadingSaving::loadXYZ("bunny/model.xyz");//.block(0, 0, 1000, 6);
    MatrixXd mSmall=PointCloudManipulation::downSample(m,false);

    Translation3d traCentroid=PointCloudManipulation::getTranslationToCentroid(mSmall);

    mSmall=PointCloudManipulation::projectPointsAndNormals(Projective3d(traCentroid),mSmall);
    
    //double diamM=PointCloudManipulation::getPointCloudDiameter(mSmall);
    
    //LoadingSaving::saveXYZ("bunny/model_downSampled.xyz", mSmall);
    //MatrixXd mSmallVoxelCenter=PointPairFeatures::downSample(m,true);
    
    //Visualize::getInstance()->ms.push_back({m,RowVector3f(1,0,0)});
    //Visualize::getInstance()->ms.push_back({mSmall,RowVector3f(1,0.5,0)});
    //Visualize::getInstance()->ms.push_back({mSmallVoxelCenter,RowVector3f(1,0.9,0)});

    MatrixXd s=LoadingSaving::loadXYZ("bunny/scene.xyz");
    MatrixXd sSmall=PointCloudManipulation::downSample(s,false);
    //LoadingSaving::saveXYZ("bunny/scene_downSampled.xyz", sSmall);

    sSmall=PointCloudManipulation::projectPointsAndNormals(Projective3d(traCentroid),sSmall);

    Quaterniond q(AngleAxisd(radians(90), Vector3d::UnitX()));
    //q = q * AngleAxisd(radians(45),Vector3d::UnitY());

    Vector4d rot(q.x(),q.y(),q.z(),q.w());
    //Vector4d rot(.2,.2,.2,.4);
    Vector3d tra(.1,0,0);

    Projective3d P = Translation3d(tra)*Quaterniond(rot);
    PointPairFeatures::printPose(P,"P_original:");


    sSmall=PointCloudManipulation::projectPointsAndNormals(P, sSmall);


    PointPairFeatures* ppfs=new PointPairFeatures();


    //Visualize::getInstance()->ms.push_back({sSmall,RowVector3f(0.5,1,0)});
    
    GlobalModelDescription map =  ppfs->buildGlobalModelDescription(mSmall);
    
    Matches matches = ppfs->matchSceneAgainstModel(sSmall, map);
    
    vector<MatrixXi> accVec=ppfs->voting(matches);
    
    //cout<<accVec[0]<<endl;
    
    Poses Pests = ppfs->computePoses(accVec, mSmall, sSmall);
    vector<Poses> clusters = ppfs->clusterPoses(Pests);
    Pests = ppfs->averagePosesInClusters(clusters);

    for(Pose PoseEst : Pests){
        ppfs->err(P,PoseEst);
    }
    
    MatrixXd modelPoseEst=PointCloudManipulation::projectPointsAndNormals(Pests[0].first, mSmall);

    PointPairFeatures::printPose(P,"P_original:");
    PointPairFeatures::printPose(Pests[0],"P_est:");

    //printMap(map);
    //KeyBucketPairList best10=PointPairFeatures::print10(map);
    
    //Visualize::getInstance()->b=best10;


    inst->model=mSmall;
    inst->scene=sSmall;
    inst->modelT=modelPoseEst;
    inst->matches=matches;

    Visualize::visualize();
}
