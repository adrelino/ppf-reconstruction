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
    
    double getPointCloudDiameter(MatrixXf m);
    MatrixXf projectPointsAndNormals(Isometry3f P, MatrixXf C);
    pair<MatrixXf,VectorXf> reestimateNormals(MatrixXf C);
    MatrixXf downSample(MatrixXf C, bool useCenter);
    MatrixXf translateCentroidToOrigin(MatrixXf C);

    Translation3f getTranslationToCentroid(MatrixXf C);

    Vector3f getCentroid(vector<Vector3f> pts);

    vector<int> getClosesPoints(MatrixXf modelPoseEst, MatrixXf sSmall, vector<Vector3f> &src, vector<Vector3f> &dst,
    float thresh);

}

namespace ICP {


    Isometry3f computeStep(vector<Vector3f> &src,vector<Vector3f> &dst,Vector3f &a,Vector3f &b,bool withScale);
    Isometry3f computeStep(vector<Vector3f> &src,vector<Vector3f> &dst,vector<Vector3f> &nor);
    Isometry3f computeStep(vector<Vector3f> &src,vector<Vector3f> &dst,bool withScale);

    //Isometry3f computeStepUnordered(MatrixXf modelPoseEst, MatrixXf sSmall, float thresh);

}

#endif /* defined(__PointPairFeatures__PointCloudManipulation__) */




