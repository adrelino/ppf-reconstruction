/**
* Copyright (C) 2014 BMW Car IT Gmbh. All rights reserved.
*
* Author: Adrian Haarbach (adrian.haarbach@bmw-carit.de)
*/

//
//  mainPoseDiff.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 15.10.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//
#include "Visualize.h"
#include "LoadingSaving.h"
#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
#include <iostream>

#define degrees(r) (180*(r)/M_PI)
#define radians(d) (M_PI*(d)/180)

using namespace std;

int main(int argc, char * argv[])
{
    Projective3d P1 = Translation3d(0.3, 0, 0) * AngleAxisd(M_2_PI, Vector3d::UnitX());// * Scaling(s);
    Projective3d P2 = Translation3d(0, 0, 0) * AngleAxisd(M_PI, Vector3d::UnitX());// * Scaling(s);


    cout<<"P1:"<<endl;
    cout<<P1.matrix()<<endl;

    cout<<"P2:"<<endl;
    cout<<P2.matrix()<<endl;


    Vector3d    tra1 = P1.translation();
    Quaterniond rot1(P1.rotation());

    Vector3d    tra2 = P2.translation();
    Quaterniond rot2(P2.rotation());


    //Translation
    double diff_tra=(tra1-tra2).norm();
    double model_diameter = 0.15; //cm
    double thresh_tra = 0.05 * model_diameter;


    //Rotation
    double d = rot1.dot(rot2);
    double diff_rot= 1 - d*d; //http://www.ogre3d.org/forums/viewtopic.php?f=10&t=79923

    double thresh_rot_degrees = 10;
    double thresh_rot = 0.25;

    double diff_rot_bertram = acos((rot1.inverse() * rot2).norm()); //bertram

    double diff_rot_degrees = degrees(acos(2*d - 1));

    cout<<"diff_rot_0to1nor\t="<<diff_rot<<endl;
    cout<<"diff_rot_bertram\t="<<diff_rot_bertram<<endl;
    cout<<"diff_rot_degrees\t="<<diff_rot_degrees<<endl;



    if(diff_tra<thresh_tra && diff_rot_degrees < thresh_rot_degrees){
        cout<<"similar Pose"<<endl;
    }
}
