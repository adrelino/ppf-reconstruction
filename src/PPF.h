//
//  PPF.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef __PointPairFeatures__PPF__
#define __PointPairFeatures__PPF__

#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class PPF {

public:

    int i,j;
    int index; //for scene reference points, so we can add it in acc array
    RowVector3d m1, m2, n1, n2;
    
    double _d,_n1d,_n2d,_n1n2;
    int d,n1d,n2d,n1n2;
    
    double alpha;
    //TODO: think about weather to save this for later reuse if ppf is peak in acc array
    //Projective3d T; //Transformation of point and normal to local coordinates, aligned with x axis

    //PPF();
    static PPF makePPF(RowVectorXd p1,RowVectorXd p2, int i, int j);
    
    void pointPairFeature(RowVector3d m1,RowVector3d m2,RowVector3d n1,RowVector3d n2);
    
    void planarRotAngle();

    static Projective3d twistToLocalCoords(Vector3d m, Vector3d n);//translates m to origin, rotates n onto x Axis
    
    void print();
    
    bool operator==(const PPF &o) const;
    
    //struct PPFsHasher{
    std::size_t operator()(const PPF& k) const;
};

#endif /* defined(__PointPairFeatures__PPF__) */
