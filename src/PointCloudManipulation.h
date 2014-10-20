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
#include "LoadingSaving.h"
#include "PointPairFeatures.h" //for ddist

using namespace Eigen;
using namespace std;


class PointCloudManipulation {
    
public:
    static double getPointCloudDiameter(MatrixXd m);
    static MatrixXd projectPointsAndNormals(Projective3d P, MatrixXd C);
    static MatrixXd reestimateNormals(MatrixXd C);
    static MatrixXd downSample(MatrixXd C, bool useCenter);
    static MatrixXd translateCentroidToOrigin(MatrixXd C);

    static Translation3d getTranslationToCentroid(MatrixXd C);

};

#endif /* defined(__PointPairFeatures__PointCloudManipulation__) */




