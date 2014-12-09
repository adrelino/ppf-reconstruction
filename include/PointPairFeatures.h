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

    Isometry3f getTransformationBetweenPointClouds(PointCloud m, PointCloud s);

    //trained model contains m, traToCentroid, as well as GlobalModelDescription (discretised ppf's buckets)
    Isometry3f getTransformationBetweenPointClouds(TrainedModel trainedModel,PointCloud s);

    //long numberOfSceneRefPts,Nm;
    //vector<int> sceneIndexToI;

    void printBucket(Bucket v);
    void printMap(GlobalModelDescription m);
    KeyBucketPairList print10(GlobalModelDescription &mymap);

    bool isPoseSimilar(Isometry3f P1, Isometry3f P2);

    bool isPoseCloseToIdentity(Isometry3f P1, float eps);

    bool isClusterSimilar(Poses cluster1, Poses cluster2);

    TrainedModel trainModel(PointCloud m);

    GlobalModelDescription buildGlobalModelDescription(PointCloud m);
    MatchesWithSceneRefIdx matchSceneAgainstModel(PointCloud s, GlobalModelDescription model);
    vector<MatrixXi> voting(MatchesWithSceneRefIdx matches, int Nm);
    Poses computePoses(vector<MatrixXi> acc,PointCloud m, PointCloud s, vector<int> sceneIndexToI);


    float getAngleDiffMod2Pi(float modelAlpha, float sceneAlpha); //always positive, needed for accumulator array discretisation

    Isometry3f alignSceneToModel(Vector3f s_m,Vector3f s_n,Vector3f m_m,Vector3f m_n,double angleAroundXAxis);


    vector<Poses> clusterPoses(Poses); //todo: several poses can be returned if severel instances of object in scene

    Pose averagePosesInCluster(Poses);
    Poses averagePosesInClusters(vector<Poses>);


//    void printPose(Pose Pest,string title="");

//    void printPose(Isometry3f P,string title="");

//    void printPoses(Poses vec);

//    float err(Isometry3f P, Pose Pest);
//    float err(Isometry3f P, Isometry3f Pest);

    Poses sortPoses(Poses poses);

    Vector4f avg_quaternion_markley(MatrixXf Q);

    Vector4f avg_quaternion_markley(Poses poses);

    Quaternionf avg_quaternion_markleyQ(Poses poses);

}

#endif /* defined(__PointPairFeatures__PointPairFeatures__) */
