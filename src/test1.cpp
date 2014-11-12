//
//  test1.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 27.06.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "PPF.h"

using namespace Eigen;

int main(int, char *[])
{
    //MatrixXf m = MatrixXf::Random(10000, 3)*0.5;
    //VectorXd indices = VectorXd::Random(0.2*800);
    
    RowVectorXf p1(6);
    p1 << 3,1,4,5,2,3;
    
    RowVectorXf p2(6);
    p2 << 2,3,6,7,1,7;
    
    PPF ppf = PPF::makePPF(p1, p2, 0, 1);
    
    ppf.planarRotAngle();

}