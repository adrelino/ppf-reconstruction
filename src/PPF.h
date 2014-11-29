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
#include "pointcloud.h"

using namespace std;
using namespace Eigen;


class PPF {

public:

    PPF();
    PPF(PointCloud C, int i, int j);

    int i,j;
    int index; //for scene reference points, so we can add it in acc array
    //RowVector3f m1, m2, n1, n2;
    
    //double _d,_n1d,_n2d,_n1n2;
    uint8_t d,n1d,n2d,n1n2;  //all of these are <255
    
    double alpha;
    //TODO: think about weather to save this for later reuse if ppf is peak in acc array
    //Projective3f T; //Transformation of point and normal to local coordinates, aligned with x axis


        
    static double planarRotAngle(Vector3f m, Vector3f n, Vector3f m2); //n2 not needed, it is automatically parallel to n because of the feature

    static Isometry3f twistToLocalCoords(Vector3f m, Vector3f n);//translates m to origin, rotates n onto x Axis
    
    void print();
    
    bool operator==(const PPF &o) const;
    int operator()(const PPF& k) const;

    int hashKey();

    static Vector4f computePPF(Vector3f m1, Vector3f n1, Vector3f m2, Vector3f n2);
};

#endif /* defined(__PointPairFeatures__PPF__) */
