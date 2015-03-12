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

#include "Params.h"
#include "LoadingSaving.h"
#include "PPF.h"
#include "Constants.h"

using namespace Eigen;
using namespace std;

namespace PointPairFeatures{

    Poses getTransformationBetweenPointClouds(PointCloud& m, PointCloud& s, bool useVersion2=true);

    bool isClusterSimilar(Poses cluster1, Poses cluster2, float thresh_rot_l=Params::getInstance()->thresh_rot, float thresh_tra_l=Params::getInstance()->thresh_tra);

    Poses computePoses(vector<MatrixXi>& acc,PointCloud& m, PointCloud& s);//, vector<int> sceneIndexToI=vector<int>(0));

    float getAngleDiffMod2Pi(float modelAlpha, float sceneAlpha); //always positive, needed for accumulator array discretisation

    Isometry3f alignModelToScene(Vector3f s_m,Vector3f s_n,Vector3f m_m,Vector3f m_n,double angleAroundXAxis);

    vector<Poses> clusterPoses(Poses, float thresh_rot_l=Params::getInstance()->thresh_rot, float thresh_tra_l=Params::getInstance()->thresh_tra); //todo: several poses can be returned if severel instances of object in scene

    vector<Pose> fromIsometry(vector<Isometry3f>& isom);

    Pose averagePosesInCluster(Poses);

    Poses averagePosesInClusters(vector<Poses>);

    Poses sortPoses(Poses poses);

    Vector4f avg_quaternion_markley(MatrixXf Q);

    Vector4f avg_quaternion_markley(Poses poses);

    Quaternionf avg_quaternion_markleyQ(Poses poses);

    void printPoses(Poses vec);
}

#endif /* defined(__PointPairFeatures__PointPairFeatures__) */
