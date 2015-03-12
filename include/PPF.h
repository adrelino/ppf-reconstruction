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
#include <vector>

#include "Params.h"


using namespace Eigen;
using namespace std;

//struct PPF2 {unsigned int k;
//            float alpha;
//            unsigned short i;
//            //unsigned short j;
//            bool operator<(const PPF2 &o) const{
//                 return k < o.k;
//            }
//            bool operator>(const PPF2 &o) const{
//                  return k > o.k;
//            }
//            bool operator==(const PPF2 &o) const{
//                   return k == o.k;
//            }
//            };


class PPF {


public:

    //static PPF2 makePPF2(const vector<Vector3f> &pts, const vector<Vector3f> &nor, int i, int j);

    PPF();
    PPF(const vector<Vector3f> &pts, const vector<Vector3f> &nor, int i, int j);

    unsigned short i;//,j;
    unsigned short index; //for scene reference points, so we can add it in acc array
    //RowVector3f m1, m2, n1, n2;
    
    //double _d,_n1d,_n2d,_n1n2;


    unsigned int k; //the hashkey
    
    float alpha;
    //TODO: think about weather to save this for later reuse if ppf is peak in acc array
    //Projective3f T; //Transformation of point and normal to local coordinates, aligned with x axis

    void print();
    
    bool operator==(const PPF &o) const;
    int operator()(const PPF& k) const;

    bool operator<(const PPF& o) const; //for sorting

    unsigned int hashKey();

    static Vector4f computePPF(Vector3f m1, Vector3f n1, Vector3f m2, Vector3f n2);


//moves p to origin, aligns n with x-axis
static
Isometry3f alignToOriginAndXAxis(Vector3f p, Vector3f n){
    Vector3f xAxis = Vector3f::UnitX();
    double angle = acos(xAxis.dot(n));
    Vector3f axis = (n.cross(xAxis)).normalized();
    //if n parallel to x axis, cross product is [0,0,0]
    if(n.y()==0 && n.z()==0) axis=Vector3f::UnitY();
    Translation3f tra(-p);
    return Isometry3f( AngleAxisf(angle, axis) * tra);
}

//n2 not needed, it is automatically parallel to n because of the feature
static
float planarRotAngle(Vector3f p_i, Vector3f n_i, Vector3f p_j){
    Isometry3f T_ms_g=alignToOriginAndXAxis(p_i,n_i);
    Vector3f p_j_image=T_ms_g*p_j;
    //can ignore x coordinate, since we rotate around x axis
    return atan2f(p_j_image.z(), p_j_image.y());
}


};

#endif /* defined(__PointPairFeatures__PPF__) */
