//
//  mainPoseDiff.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 15.10.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "LoadingSaving.h"
#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
#include <iostream>
#include "RandomN.h"

using namespace std;

int main(int argc, char * argv[])
{
    Isometry3f P1 = Translation3f(0, 0, 1) * AngleAxisf(M_PI, Vector3f::UnitX()+Vector3f::UnitY()*0.05);// * Scaling(s);
    Isometry3f P2 = Translation3f(0, 0, 0.9999) * AngleAxisf(M_PI, Vector3f::UnitX()-Vector3f::UnitY()*0.05);// * Scaling(s);
    Isometry3f P3 = Translation3f(0, 0, 1.0001) * AngleAxisf(M_PI+2, Vector3f::UnitX());// * Scaling(s);

    cout<<"P1:"<<endl;
    cout<<P1.linear().matrix()<<endl;

    cout<<"P2:"<<endl;
    cout<<P2.linear().matrix()<<endl;

    cout<<"P3:"<<endl;
    cout<<P3.linear().matrix()<<endl;

    cout<<"P1.equals(P2): "; PointPairFeatures::isPoseSimilar(P1,P2);
    cout<<"P1.equals(P3): "; PointPairFeatures::isPoseSimilar(P1,P3);
    cout<<"P2.equals(P3): "; PointPairFeatures::isPoseSimilar(P2,P3);

    Poses vec;
    vec.push_back(make_pair(P1,5));
    vec.push_back(make_pair(P2,10));
    vec.push_back(make_pair(P3,1));

    vector<Poses> clusters = PointPairFeatures::clusterPoses(vec);

    Poses posesAveraged = PointPairFeatures::averagePosesInClusters(clusters);

    PointPairFeatures::printPoses(posesAveraged);



cout<<endl<<endl<<endl;
    //adding small uniform/gaussian noise to quaternion and translation
    //http://www.mathworks.com/matlabcentral/fileexchange/40098-averaging-quaternions
    int nSamples = 100;
    double sigma = 0.9;


    Quaternionf q(AngleAxisf(deg2rad(45), Vector3f::UnitX()));
    q = q * AngleAxisf(deg2rad(45),Vector3f::UnitY());

    Vector4f rot(q.x(),q.y(),q.z(),q.w());
    Vector3f tra(0.1,0,0);

    Isometry3f P = Translation3f(tra)*Quaternionf(rot);

    Poses cluster;

    for (int i = 0; i < nSamples; ++i) {
        //Vector4f rotRand=RandomN::RandomNGet(4); //standard normally
        Vector4f rotRand=Vector4f::Random(); // uniform(-1,1) distributed samples
        //Vector4f rotTest = rot;
        //if(i % 3 == 1) rotTest=-rotTest;
        Vector4f rotNoisy = rot + sigma * rotRand;
        //Vector3f traRand=RandomN::RandomNGet(3);//standard normally
        Vector3f traRand=Vector3f::Random(); // uniform(-1,1) distributed samples
        Vector3f traNoisy = tra + sigma * traRand;

        //cout<<"traRand: "<<traRand.transpose()<<endl;
        //cout<<"rotRand: "<<rotRand.transpose()<<endl;

        Isometry3f Pi=Translation3f(traNoisy)*Quaternionf(rotNoisy);
        cluster.push_back(std::make_pair(Pi,1));
    }

    Pose PoseEst = PointPairFeatures::averagePosesInCluster(cluster);

    PointPairFeatures::err(P,PoseEst);



}
