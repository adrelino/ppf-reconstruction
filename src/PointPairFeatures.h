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


class PointPairFeatures{
    public:
    
    long numberOfSceneRefPts,Nm;
    vector<int> sceneIndexToI;

    static void printBucket(Bucket v);
    static void printMap(GlobalModelDescription m);
    static KeyBucketPairList print10(GlobalModelDescription &mymap);

    static bool isPoseSimilar(Projective3d P1, Projective3d P2);
    static bool isClusterSimilar(Poses cluster1, Poses cluster2);

    
    GlobalModelDescription buildGlobalModelDescription(MatrixXd m);
    Matches matchSceneAgainstModel(MatrixXd m, GlobalModelDescription model);
    vector<MatrixXi> voting(Matches matches);
    
    Poses computePoses(vector<MatrixXi> acc,MatrixXd m, MatrixXd s);
    static vector<Poses> clusterPoses(Poses); //todo: several poses can be returned if severel instances of object in scene

    static Pose averagePosesInCluster(Poses);
    static Poses averagePosesInClusters(vector<Poses>);

    static void printPoses(Poses vec);

    static void err(Projective3d P, Pose Pest);
    static void err(Projective3d P, Projective3d Pest);

};

#endif /* defined(__PointPairFeatures__PointPairFeatures__) */
