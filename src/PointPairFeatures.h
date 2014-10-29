//
//  PointPairFeatures.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.07.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef __PointPairFeatures__PointPairFeatures__
#define __PointPairFeatures__PointPairFeatures__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

#include "LoadingSaving.h"
#include "PPF.h"
#include "Constants.h"


using namespace Eigen;
using namespace std;


namespace PointPairFeatures{

    Projective3d getTransformationBetweenPointClouds(MatrixXd m, MatrixXd s);

    Projective3d getTransformationBetweenPointClouds(MatrixXd m, MatrixXd s, GlobalModelDescription model);


    
    //long numberOfSceneRefPts,Nm;
    //vector<int> sceneIndexToI;

    void printBucket(Bucket v);
    void printMap(GlobalModelDescription m);
    KeyBucketPairList print10(GlobalModelDescription &mymap);

    bool isPoseSimilar(Projective3d P1, Projective3d P2);
    bool isClusterSimilar(Poses cluster1, Poses cluster2);

    
    GlobalModelDescription buildGlobalModelDescription(MatrixXd m);
    MatchesWithSceneRefIdx matchSceneAgainstModel(MatrixXd s, GlobalModelDescription model);
    vector<MatrixXi> voting(MatchesWithSceneRefIdx matches, int Nm);
    Poses computePoses(vector<MatrixXi> acc,MatrixXd m, MatrixXd s, vector<int> sceneIndexToI);


    double getAngleDiffMod2Pi(double modelAlpha, double sceneAlpha); //always positive, needed for accumulator array discretisation

    Projective3d alignSceneToModel(RowVectorXd sceneRefPt, RowVectorXd modelRefPt, double angleAroundXAxis);


    vector<Poses> clusterPoses(Poses); //todo: several poses can be returned if severel instances of object in scene

    Pose averagePosesInCluster(Poses);
    Poses averagePosesInClusters(vector<Poses>);


    void printPose(Pose Pest,string title="");

    void printPose(Projective3d P,string title="");

    void printPoses(Poses vec);

    void err(Projective3d P, Pose Pest);
    void err(Projective3d P, Projective3d Pest);

    Poses sortPoses(Poses poses);

}

#endif /* defined(__PointPairFeatures__PointPairFeatures__) */
