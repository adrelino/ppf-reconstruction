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

#include "OpenCVHelpers.h"

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

    stringstream ss;
    ss<<"000000"<<endl;
    std::string dir = "bunny/Bunny_RealData";
    //std::string dir = "bunny/";///Bunny_Sphere";

    OpenCVHelpers::getParam("dir", dir, argc, argv);


    //cv::Mat depth = cv::imread( dir + "/depth_" + ss.str() + ".exr",IMREAD_GRAYSCALE);


    vector<string> images = OpenCVHelpers::getAllImagesFromFolder(dir,"depth");
    vector<string> clouds = OpenCVHelpers::getAllTextFilesFromFolder(dir,"cloudXYZ");

    std::cout<<" #images"<<images.size();


    //std::string filename = "/bunny/Bunny_Sphere/depth_000000.exr";

    int i=0;
    for(std::string filename : images){
        cout<<filename;

    std::ifstream ifs(filename.c_str());

    if (!ifs.is_open())
    {
      printf("Cannot open file...\n");
    }

    cv::Mat depth = cv::imread(filename,IMREAD_UNCHANGED);
    int type=depth.type();
    int nc=depth.channels();
    //cout<<"type: "<<OpenCVHelpers::getImageType(type)<<" channels:"<<nc<<endl;

    cv::Mat mask; //is 1 at the object, 0 outside

    if(type==CV_16U && nc==1){ //depth_0.png from kinect cam, 0 means no measure, depth in mm (470 means 0.47m);
        mask = (depth != 0);
    }else if(type==CV_32FC3 && nc==3){ //depth_000000.exr made from blender, inf means no measure, depth in mm
        cv::cvtColor(depth,depth,COLOR_BGR2GRAY);
        mask = (depth != std::numeric_limits<float>::infinity());
    }else{
        cout<<"unsupported depth image type"<<endl;
    }

    depth.convertTo( depth, CV_32FC1, 0.001); // convert to meters





    //cout<<"mask type: "<<OpenCVHelpers::getImageType(mask.type())<<" channels:"<<mask.channels()<<endl;


    //OpenCVHelpers::showImage("mask",mask);



    //OpenCVHelpers::showImage("showImage",depth);

    //OpenCVHelpers::imagesc("imagesc",depth,mask);

    //cv::cvtColor(depth,depth,CV_RGB2GRAY);   // like single-channel png's





    //It is now straightforward to create masks for the objects:

    //cv::Mat mask = ( depth < 10 );


    OpenCVHelpers::showDepthImage("showDepthImage",depth,mask,true);
    std::cout<<std::endl;

    cv::waitKey(2);





    string cloudName = clouds[i++];

    PointCloud CGroundTruth=LoadingSaving::loadPointCloud(cloudName);
    PointCloud CGroundTruth_small = PointCloudManipulation::downSample(CGroundTruth,ddist);
    PointCloudManipulation::reestimateNormals(CGroundTruth_small,ddist);

    PointCloud C_small = PointCloudManipulation::downSample(C,ddist);
    PointCloudManipulation::reestimateNormals(C_small,ddist);

    Visualize::setScene(C_small);
    Visualize::setModel(CGroundTruth_small);

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
