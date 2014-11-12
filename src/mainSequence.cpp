//
//  mainSequence.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 29.10.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "LoadingSaving.h"
//#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
//#include <iostream>
//#include <string>
//#include "Constants.h"

#include <opencv2/viz/vizcore.hpp>
#include <opencv2/viz/viz3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;

cv::Mat eig2cv(MatrixXf C)
{
    cv::Mat cloud(1, C.rows(), CV_32FC3);

    cv::Point3f* data = cloud.ptr<cv::Point3f>();

    for (int i=0; i<C.rows(); i++) {
        RowVector3f p1=C.block(i, 0, 1, 3);

        data[i].x=p1(0);
        data[i].y=p1(1);
        data[i].z=p1(2);

        //cout<<cloud.at<cv::Point3f>(0,i)<<endl;
    }



    return cloud;
}

int main(int argc, char * argv[])
{
    cv::viz::Viz3d myWindow("mainSequence");
    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());


    cv::viz::Widget cloudWidget=cv::viz::Widget::fromPlyFile("bunny/bunny.ply");
    myWindow.showWidget("Cloud", cloudWidget);

    MatrixXf cloud;

    vector<cv::Affine3f> poses;

    for(int i=0; i<=36 && !myWindow.wasStopped(); i++){

        stringstream ss,ss1,ss2;
        ss<<"bunny/depth-cloud/cloudXYZ_"<<i<<".xyz";
        cloud=LoadingSaving::loadMatrixXf(ss.str());
        //cout<<cloud<<endl;

        ss1<<"bunny/depth-poses/poses_"<<i<<".txt";
        Isometry3f P(LoadingSaving::loadMatrix4f(ss1.str()));
        P = AngleAxisf(deg2rad(-90), Vector3f::UnitX())*P;

        ss2.flush();
        ss2<<"P:"<<i<<endl;


//        PointPairFeatures::printPose(P,ss2.str());

        cloud=PointCloudManipulation::projectPointsAndNormals(P, cloud);

        //MatrixXf

        //cv::Mat B_OpenCV(cloud.rows(), cloud.cols(), CV_32FC1, cloud.data());

        cv::Mat cloudCv = eig2cv(cloud);
//        Matrix4f PRowMajor(RowMajor);
//        PRowMajor(P.matrix());
        cv::Mat PCv;
        cv::eigen2cv(P.matrix(),PCv);
        cv::Affine3f Pcv(PCv);

        cout<<"eig"<<P.matrix()<<endl;
        cout<<"cvRot"<<Pcv.rotation()<<endl;
        cout<<"cvTra"<<Pcv.translation()<<endl;

        poses.push_back(Pcv);

        cv::viz::WTrajectory traj(poses);

        myWindow.showWidget("poses", traj);





        cv::viz::WPaintedCloud cloudWidget(cloudCv);
        myWindow.showWidget(ss2.str(), cloudWidget);




        myWindow.spinOnce(30,true);

    }

    while(!myWindow.wasStopped()){
        myWindow.spinOnce(30,true);

    }

    exit(0);
    myWindow.close();

}
