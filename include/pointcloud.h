#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <eigen3/Eigen/Dense>
#include <vector>
using namespace Eigen;
using namespace std;

#include <opencv2/core.hpp>

//typedef struct {float x; float y; float z;} float3;
//inline std::ostream& operator<<(std::ostream& os, const float3& v) {
//  os << v.x << " " << v.y << " " << v.z << std::endl;
//  return os;
//}
//typedef std::pair< std::vector<Vector3f>, std::vector<Vector3f> > PointCloud;

//https://forum.kde.org/viewtopic.php?f=74&t=94839
static Matrix3Xf vec2mat(vector<Vector3f> vec){
    Map<Matrix<float,3,Dynamic,ColMajor> > mat(&vec[0].x(), 3, vec.size());
    return mat;
}

static vector<Vector3f> mat2vec(Matrix3Xf mat){
    vector<Vector3f> vec;
    for(int i=0; i<mat.cols(); i++){
        vec.push_back(mat.col(i));
    }

    //http://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
    //vector<Vector3f> vec(mat.data(),mat.data() + mat.rows() * mat.cols());
    return vec;
}



class PointCloud
{
public:
    PointCloud(){};
    //PointCloud(PointCloud &c);

//    PointCloud(int n){
//        pts = std::vector<Vector3f>(n);
//        nor = std::vector<Vector3f>(n);
//        pts_color = std::vector<RowVector3f>(n);
//    }

    static PointCloud fromDepthImage(cv::Mat depth, cv::Mat mask){
        //mimic depth2cloudAndNormals.m matlab file

        int m=depth.rows; //Y
        int n=depth.cols; //X

        float fx = 542;
        float fy=540.5;
        float cx=n/2;
        float cy=m/2;



        vector<Eigen::Vector3f> pts;

        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                if(mask.at<bool>(i,j)){
                    float Z = depth.at<float>(i,j);
                    float X = (j-cx)*Z*(1/fx);
                    float Y = (i-cy)*Z*(1/fy);
                    pts.push_back(Vector3f(X,Y,Z));
                }
            }
        }

        PointCloud C;
        C.pts=pts;

        return C;
    }

    static PointCloud fromOther(PointCloud &c){  //copy constructor
        PointCloud a;
        a.pts=c.pts;
        a.nor=c.nor;
        a.pts_color=c.pts_color;
        return a;
    }

//    PointCloud& operator=(PointCloud other) {
//        std::cout << "copy assignment of A\n";
//        std::swap(pts, other.pts);
//        std::swap(nor, other.nor);
//        return *this;
//    }

//        PointCloud& operator=() {
//            std::cout << "copy assignment of A\n";
//            std::swap(pts, other.pts);
//            std::swap(nor, other.nor);
//            return *this;
//        }

    std::vector<Vector3f> pts;
    std::vector<Vector3f> nor;
//    std::vector<float> cur;
    std::vector<RowVector3f> pts_color;
//    float neighRadius;

    int rows(){
        return pts.size();
    }

    Matrix3Xf ptsMat(){
        return vec2mat(pts);
    }

    Matrix3Xf norMat(){
        return vec2mat(nor);
    }

    void append(PointCloud b){
        pts.insert(pts.end(), b.pts.begin(), b.pts.end());
        nor.insert(nor.end(), b.nor.begin(), b.nor.end());
        pts_color.insert(pts_color.end(), b.pts_color.begin(), b.pts_color.end());


    }
};

//typedef struct { std::vector<Vector3f> pts; std::vector<Vector3f> nor; std::vector<float> cur; std::vector<RowVector3f> pts_color; float neighRadius;} PointCloud;
inline std::ostream& operator<<(std::ostream& os, const PointCloud& v) {
  os << "size: " <<  v.pts.size() <<std::endl \
     << "pts:" << std::endl << vec2mat(v.pts) <<std::endl \
     << "nor:" << std::endl << vec2mat(v.nor)<< std::endl;
  return os;
}

#endif // POINTCLOUD_H
