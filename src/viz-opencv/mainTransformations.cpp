/**
 * @file transformations.cpp
 * @brief Visualizing cloud in different positions, coordinate frames, camera frustums
 * @author Ozan Cagri Tonkal
 */

#include <opencv2/viz/vizcore.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include "pointcloud.h"
#include "Visualize.h"
#include "LoadingSaving.h"

#include "PointCloudManipulation.h"

using namespace cv;
using namespace std;

///**
// * @function help
// * @brief Display instructions to use this tutorial program
// */
//void help()
//{
//    cout
//    << "--------------------------------------------------------------------------"   << endl
//    << "This program shows how to use makeTransformToGlobal() to compute required pose,"
//    << "how to use makeCameraPose and Viz3d::setViewerPose. You can observe the scene "
//    << "from camera point of view (C) or global point of view (G)"                    << endl
//    << "Usage:"                                                                       << endl
//    << "./transformations [ G | C ]"                                                 << endl
//    << endl;
//}

///**
// * @function cvcloud_load
// * @brief load bunny.ply
// */
//Mat cvcloud_load()
//{
//    Mat cloud(1, 3, CV_32FC3);
//    ifstream ifs("bunny/bunny.ply");

//    string str;
//    for(size_t i = 0; i < 12; ++i)
//        getline(ifs, str);

//    Point3f* data = cloud.ptr<cv::Point3f>();
//    float dummy1, dummy2;
//    for(size_t i = 0; i < 1889; ++i)
//        ifs >> data[i].x >> data[i].y >> data[i].z >> dummy1 >> dummy2;

//    cloud *= 5.0f;
//    return cloud;
//}

/**
 * @function main
 */
int main(int argc, char **argv)
{

    //bool camera_pov = false; //(argv[1][0] == 'C');

    /// Create a window
    //viz::Viz3d myWindow("Coordinate Frame");

    /// Add coordinate axes
    //myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());

    /// Let's assume camera has the following properties
   // Vec3d cam_pos(3.0f,3.0f,3.0f), cam_focal_point(3.0f,3.0f,2.0f), cam_y_dir(-1.0f,0.0f,0.0f);

    /// We can get the pose of the cam using makeCameraPose
    //Affine3d cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

    /// We can get the transformation matrix from camera coordinate system to global using
    /// - makeTransformToGlobal. We need the axes of the camera
    //Affine3d transform = viz::makeTransformToGlobal(Vec3f(0.0f,-1.0f,0.0f), Vec3f(-1.0f,0.0f,0.0f), Vec3f(0.0f,0.0f,-1.0f), cam_pos);

    /// Create a cloud widget.
    //M//at bunny_cloud = cvcloud_load();
   // cout<<bunny_cloud<<endl;

    //viz::WCloud cloud_widget(bunny_cloud, viz::Color::green());

    //std::string dir = "bunny/Bunny_RealData";
    std::string dir = "bunny/Bunny_Sphere";

    getParam("dir", dir, argc, argv);

    vector<string> images = LoadingSaving::getAllImagesFromFolder(dir,"depth");
    vector<string> clouds = LoadingSaving::getAllTextFilesFromFolder(dir,"cloudXYZ");

    bool hasClouds = clouds.size()==images.size();

    std::cout<<"#images:"<<images.size()<<" #clouds:"<<clouds.size();

    vector<string> intrinsics = LoadingSaving::getAllTextFilesFromFolder(dir,"Intrinsic");

    Matrix3f K = LoadingSaving::loadMatrix3f(intrinsics[0]);

    int i=0;
    for(std::string filename : images){
        cout<<filename;



        PointCloud C = LoadingSaving::loadPointCloudFromDepthMap(filename,K,true);

        if(hasClouds){
            string cloudName = clouds[i++];
            PointCloud CGroundTruth=LoadingSaving::loadPointCloud(cloudName);
            //PointCloud CGroundTruth_small = PointCloudManipulation::downSample(CGroundTruth,ddist);
            //PointCloudManipulation::reestimateNormals(CGroundTruth_small,ddist);
            Visualize::setModel(CGroundTruth);
        }

        //PointCloud C_small = PointCloudManipulation::downSample(C,ddist);
        //PointCloudManipulation::reestimateNormals(C_small,ddist);

        Visualize::setScene(C);


        //stringstream ss;
        //ss<<cloudName<<".comp";
        //LoadingSaving::writePointCloud(ss.str(),C);



        Visualize::spin();
    }




    /// Pose of the widget in camera frame
    //Affine3d cloud_pose = Affine3d().translate(Vec3d(0.0f,0.0f,3.0f));
    /// Pose of the widget in global frame
   // Affine3d cloud_pose_global = transform * cloud_pose;

//    /// Visualize camera frame
//    if (!camera_pov)
//    {
//        viz::WCameraPosition cpw(0.5); // Coordinate axes
//        viz::WCameraPosition cpw_frustum(Vec2f(0.889484, 0.523599)); // Camera frustum
//        myWindow.showWidget("CPW", cpw, cam_pose);
//        myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
//    }

//    /// Visualize widget
//    myWindow.showWidget("bunny", cloud_widget, cloud_pose_global);



    /// Set the viewer pose to that of cameraQq
    //if (camera_pov)
    //    myWindow.setViewerPose(cam_pose);

    /// Start event loop.
    //myWindow.spin();

    return 0;
}
