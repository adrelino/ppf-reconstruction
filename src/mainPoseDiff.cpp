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

using namespace std;

int main(int argc, char * argv[])
{
    Projective3d P1 = Translation3d(0, 0, 0) * AngleAxisd(M_PI, Vector3d::UnitX()+Vector3d::UnitY()*0.1);// * Scaling(s);
    Projective3d P2 = Translation3d(0, 0, 0) * AngleAxisd(M_PI, Vector3d::UnitX());// * Scaling(s);
    Projective3d P3 = Translation3d(0, 0, 0) * AngleAxisd(M_PI+2, Vector3d::UnitX());// * Scaling(s);


    cout<<"P1:"<<endl;
    cout<<P1.rotation().matrix()<<endl;

    cout<<"P2:"<<endl;
    cout<<P2.rotation().matrix()<<endl;

    cout<<"P3:"<<endl;
    cout<<P3.rotation().matrix()<<endl;

    cout<<"P1.equals(P2): "; PointPairFeatures::isPoseSimilar(P1,P2);
    cout<<"P1.equals(P3): "; PointPairFeatures::isPoseSimilar(P1,P3);
    cout<<"P2.equals(P3): "; PointPairFeatures::isPoseSimilar(P2,P3);



    //Projective3d P4 = Translation3d(0, 0, 0) * AngleAxisd(0.5, Vector3d::UnitX()+Vector3d::UnitY()*0.1);// * Scaling(s);
    //Projective3d P5 = Translation3d(0, 0, 0) * AngleAxisd(0.5, Vector3d::UnitX());// * Scaling(s);

    Poses vec;
    vec.push_back(make_pair(P1,5));
    vec.push_back(make_pair(P2,10));
    vec.push_back(make_pair(P3,1));

    Pose Pest = PointPairFeatures::clusterPoses(vec);

    cout<<"P_aver with score of :"<<Pest.second<<endl;
    cout<<Pest.first.rotation().matrix()<<endl;





}
