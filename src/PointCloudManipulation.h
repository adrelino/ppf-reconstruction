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
    
    double getPointCloudDiameter(MatrixXd m);
    MatrixXd projectPointsAndNormals(Projective3d P, MatrixXd C);
    MatrixXd reestimateNormals(MatrixXd C);
    MatrixXd downSample(MatrixXd C, bool useCenter);
    MatrixXd translateCentroidToOrigin(MatrixXd C);

    Translation3d getTranslationToCentroid(MatrixXd C);

}

namespace ICP {
    Isometry3f computeStep(vector<Vector3f> &src,vector<Vector3f> &dst,Vector3f &a,Vector3f &b,bool withScale);
    Isometry3f computeStep(vector<Vector3f> &src,vector<Vector3f> &dst,vector<Vector3f> &nor);
}

#endif /* defined(__PointPairFeatures__PointCloudManipulation__) */




