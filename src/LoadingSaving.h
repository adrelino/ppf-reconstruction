//
//  LoadingSaving.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 27.06.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef __PointPairFeatures__LoadingSaving__
#define __PointPairFeatures__LoadingSaving__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
using namespace Eigen;
using namespace std;

class LoadingSaving {
    
    
public:
    static MatrixXd loadXYZ(std::string filename);
    static Matrix4d loadProjectionMatrix(std::string filename);

    static void saveXYZ(std::string filename, MatrixXd pts);
    static void saveVector(std::string filename, vector<double> vec);
    
    static void summary(std::vector<double> v); //Like R's summary

};

#endif /* defined(__PointPairFeatures__LoadingSaving__) */
