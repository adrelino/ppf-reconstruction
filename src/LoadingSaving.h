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

namespace LoadingSaving {
    using namespace Eigen;
    using namespace std;

    MatrixXi loadMatrixXi(std::string filename);
    MatrixXf loadMatrixXf(std::string filename);
    MatrixXd loadMatrixXd(std::string filename);

    Matrix4f loadMatrix4f(std::string filename);


    void saveMatrixXi(std::string filename, MatrixXi mat);
    void saveMatrixXf(std::string filename, MatrixXf mat);
    void saveMatrixXd(std::string filename, MatrixXd mat);

    void saveMatrix4f(std::string filename, Matrix4f mat);

    void saveVector(std::string filename, vector<double> vec);
}

#endif /* defined(__PointPairFeatures__LoadingSaving__) */
