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

#include <opencv2/highgui/highgui.hpp>

using namespace std;

cv::Mat eig2cv(PointCloud C, bool normals=false)
{
    cv::Mat cloud(1, C.pts.size(), CV_32FC3);

    cv::Point3f* data = cloud.ptr<cv::Point3f>();
    vector<Vector3f> vec = C.pts;
    if(normals){
        vec=C.nor;
    }

    for (int i=0; i<vec.size(); i++) {
        Vector3f p1=vec[i];

        data[i].x=p1(0);
        data[i].y=p1(1);
        data[i].z=p1(2);

        //cout<<cloud.at<cv::Point3f>(0,i)<<endl;
    }



    return cloud;
}

bool isRunning=true;
int i=0;
cv::viz::Viz3d myWindow;

void KeyboardCallbackFunction(const cv::viz::KeyboardEvent& e, void* cookie){
    cout<<"action="<<e.action<<" code="<<e.code<<" modifiers="<<e.modifiers<<" symbol="<<e.symbol<<endl;
    if(e.symbol=="w" || e.symbol == "W"){
        isRunning=false;
    }
    if(e.symbol=="N" || e.symbol == "n"){
        for(int j=0; j<i; j++){
            stringstream ss2;
            ss2<<"P:"<<i<<endl;
            myWindow.removeWidget(ss2.str()+"normals");
        }
    }
    if(e.symbol=="o"){
         myWindow.removeWidget("origin");
    }

}

void MouseCallbackFunction(const cv::viz::MouseEvent& e, void* cookie){
    cout<<"type="<<e.type<<" modifiers="<<e.modifiers<<" pointer="<<e.pointer<<" button="<<e.button<<endl;
}

int main(int argc, char * argv[])
{
    myWindow = cv::viz::Viz3d("mainSequence");
    myWindow.showWidget("origin", cv::viz::WCoordinateSystem());


    cv::viz::Widget cloudWidget=cv::viz::Widget::fromPlyFile("bunny/bunny.ply");
    myWindow.showWidget("Cloud", cloudWidget);

    PointCloud cloud;

    vector<cv::Affine3f> poses;

    myWindow.registerKeyboardCallback(KeyboardCallbackFunction);
    myWindow.registerMouseCallback(MouseCallbackFunction);

    cv::Matx33d K = cv::Matx33d::zeros();
    K(0,0) = 542;
    K(1,1) = 540;
    K(0,2) = 320;
    K(1,2) = 240;


    for(i=0; i<=36 && !myWindow.wasStopped() && isRunning; i++){

        stringstream ss,ss1,ss2,ss3;
        ss<<"bunny/depth-cloud/cloudXYZ_"<<i<<".xyz";
        cloud=LoadingSaving::loadPointCloud(ss.str());
        //cout<<cloud<<endl;

        ss1<<"bunny/depth-poses/cloudToPLY-coarse_"<<i<<".txt";
        Isometry3f P(LoadingSaving::loadMatrix4f(ss1.str()));
        //P = AngleAxisf(deg2rad(-90), Vector3f::UnitX())*P;

        ss2<<"P:"<<i<<endl;

        ss3<<"bunny/depth-imgs/mask_"<<i<<".png";

        cv::Mat image = cv::imread(ss3.str());

       // PointPairFeatures::printPose(P,ss2.str());

        cloud=PointCloudManipulation::downSample(cloud,0.01f);

        cloud=PointCloudManipulation::projectPointsAndNormals(P, cloud);

        cv::Mat PCv;
        cv::eigen2cv(P.matrix(),PCv);
        cv::Affine3f Pcv(PCv);

        //cout<<"eig"<<P.matrix()<<endl;
        //cout<<"cvRot"<<Pcv.rotation()<<endl;
        //cout<<"cvTra"<<Pcv.translation()<<endl;

        poses.push_back(Pcv);

        cv::viz::WTrajectory traj(poses);

        cv::viz::WTrajectoryFrustums trajFrust(poses,K,0.03);

        cv::viz::WCameraPosition camPos(K,image,0.05);
        //cv::viz::WCoordinateSystem camPos(0.1);
        camPos.applyTransform(Pcv);
        //camPos.setPose(Pcv.inv());


        myWindow.showWidget(ss3.str(), camPos);

        myWindow.showWidget("poses", traj);




        cv::Mat cloudCv = eig2cv(cloud);
        cv::viz::WPaintedCloud cloudWidget(cloudCv);
        myWindow.showWidget(ss2.str(), cloudWidget);

        //cv::Mat normalsCv = eig2cv(cloud,true);
        //cv::viz::WCloudNormals cloudNormals(cloudCv,normalsCv,1,0.03,cv::viz::Color::cyan());
        //myWindow.showWidget("normals",cloudNormals);



//        while(!myWindow.wasStopped()){
            myWindow.spinOnce(1,true);
//        }
//        while(!myWindow.wasStopped() && isRunning){
//            myWindow.spinOnce();
//        }
    }

    myWindow.spin();

    myWindow.close();
    cout << "Last event loop is over" << endl;
    return 0;

}
