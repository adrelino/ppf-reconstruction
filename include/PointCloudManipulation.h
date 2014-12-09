//
//  PointCloudManipulation.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef __PointPairFeatures__PointCloudManipulation__
#define __PointPairFeatures__PointCloudManipulation__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <unordered_map>
#include <vector>
#include "Constants.h"

using namespace Eigen;
using namespace std;


namespace PointCloudManipulation {
    
    double getPointCloudDiameter(PointCloud m);

    PointCloud projectPointsAndNormals(Isometry3f P, PointCloud C);

    Matrix3f covarianceOfNeighbours(const vector<Vector3f> pts, const Vector3f p1, const float neighRadius);

    vector<Vector3f> estimateNormals(const vector<Vector3f> pts, const vector<Vector3f> oldNormals, const float neighRadius);

    void reestimateNormals(PointCloud &C, const float neighRadius);

    PointCloud downSample(PointCloud C, float voxelSize);

    Translation3f getTranslationToCentroid(PointCloud C);

    Vector3f getCentroid(vector<Vector3f> pts);

    int nearestNeighbourIdx(vector<Vector3f> pts, Vector3f pt);

    vector<int> getClosesPoints(PointCloud modelPoseEst, PointCloud sSmall, vector<Vector3f> &src, vector<Vector3f> &dst,
    float thresh);

}

namespace ICP {


    Isometry3f computeStep(vector<Vector3f> &src,vector<Vector3f> &dst,Vector3f &a,Vector3f &b,bool withScale);
    Isometry3f computeStep(vector<Vector3f> &src,vector<Vector3f> &dst,vector<Vector3f> &nor);
    Isometry3f computeStep(vector<Vector3f> &src,vector<Vector3f> &dst,bool withScale);

    //Isometry3f computeStepUnordered(MatrixXf modelPoseEst, MatrixXf sSmall, float thresh);

    Isometry3f getTransformationBetweenPointClouds(PointCloud modelEst, PointCloud scene, int maxiter=100, float eps = 1e-3);

}

#endif /* defined(__PointPairFeatures__PointCloudManipulation__) */




