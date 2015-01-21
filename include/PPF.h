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

using namespace Eigen;
using namespace std;

struct PPF2 {unsigned int k;
            float alpha;
            unsigned short i;
            //unsigned short j;
            bool operator<(const PPF2 &o) const{
                 return k < o.k;
            }
            bool operator>(const PPF2 &o) const{
                  return k > o.k;
            }
            bool operator==(const PPF2 &o) const{
                   return k == o.k;
            }
            };


class PPF {


public:

    static PPF2 makePPF2(const vector<Vector3f> &pts, const vector<Vector3f> &nor, int i, int j);

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


        
    static float planarRotAngle(Vector3f m, Vector3f n, Vector3f m2); //n2 not needed, it is automatically parallel to n because of the feature

    static Isometry3f twistToLocalCoords(Vector3f m, Vector3f n);//translates m to origin, rotates n onto x Axis
    
    void print();
    
    bool operator==(const PPF &o) const;
    int operator()(const PPF& k) const;

    bool operator<(const PPF& o) const; //for sorting

    unsigned int hashKey();

    static Vector4f computePPF(Vector3f m1, Vector3f n1, Vector3f m2, Vector3f n2);
};

#endif /* defined(__PointPairFeatures__PPF__) */
