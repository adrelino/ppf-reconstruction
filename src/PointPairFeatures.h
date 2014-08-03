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
    
    GlobalModelDescription buildGlobalModelDescription(MatrixXd m);
    Matches matchSceneAgainstModel(MatrixXd m, GlobalModelDescription model);
    vector<MatrixXi> voting(Matches matches);
    
    vector<pair<Projective3d,int>> computePoses(vector<MatrixXi> acc,MatrixXd m, MatrixXd s);
    Projective3d clusterPoses(vector<pair<Projective3d,int>>); //todo: several poses can be returned if severel instances of object in scene


    
};

#endif /* defined(__PointPairFeatures__PointPairFeatures__) */
