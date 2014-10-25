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
#include "RandomN.h"

using namespace std;

int main(int argc, char * argv[])
{
    Projective3d P1 = Translation3d(0, 0, 1) * AngleAxisd(M_PI, Vector3d::UnitX()+Vector3d::UnitY()*0.05);// * Scaling(s);
    Projective3d P2 = Translation3d(0, 0, 0.9999) * AngleAxisd(M_PI, Vector3d::UnitX()-Vector3d::UnitY()*0.05);// * Scaling(s);
    Projective3d P3 = Translation3d(0, 0, 1.0001) * AngleAxisd(M_PI+2, Vector3d::UnitX());// * Scaling(s);

    cout<<"P1:"<<endl;
    cout<<P1.rotation().matrix()<<endl;

    cout<<"P2:"<<endl;
    cout<<P2.rotation().matrix()<<endl;

    cout<<"P3:"<<endl;
    cout<<P3.rotation().matrix()<<endl;

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
    double sigma = 0.1;


    Quaterniond q(AngleAxisd(radians(45), Vector3d::UnitX()));
    q = q * AngleAxisd(radians(45),Vector3d::UnitY());

    Vector4d rot(q.x(),q.y(),q.z(),q.w());
    Vector3d tra(0.1,0,0);

    Projective3d P = Translation3d(tra)*Quaterniond(rot);

    Poses cluster;

    for (int i = 0; i < nSamples; ++i) {
        //Vector4d rotRand=RandomN::RandomNGet(4); //standard normally
        Vector4d rotRand=Vector4d::Random(); // uniform(-1,1) distributed samples
        //Vector4d rotTest = rot;
        //if(i % 3 == 1) rotTest=-rotTest;
        Vector4d rotNoisy = rot + sigma * rotRand;
        //Vector3d traRand=RandomN::RandomNGet(3);//standard normally
        Vector3d traRand=Vector3d::Random(); // uniform(-1,1) distributed samples
        Vector3d traNoisy = tra + sigma * traRand;

        //cout<<"traRand: "<<traRand.transpose()<<endl;
        //cout<<"rotRand: "<<rotRand.transpose()<<endl;

        Projective3d Pi=Translation3d(traNoisy)*Quaterniond(rotNoisy);
        cluster.push_back(std::make_pair(Pi,1));
    }

    Pose PoseEst = PointPairFeatures::averagePosesInCluster(cluster);

    PointPairFeatures::err(P,PoseEst);



}
