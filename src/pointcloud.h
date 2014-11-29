#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <eigen3/Eigen/Dense>
#include <vector>
using namespace Eigen;
using namespace std;

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
    PointCloud();

    std::vector<Vector3f> pts;
    std::vector<Vector3f> nor;
    std::vector<float> cur;
    std::vector<RowVector3f> pts_color;
    float neighRadius;

    int rows(){
        return pts.size();
    }

    Matrix3Xf ptsMat(){
        return vec2mat(pts);
    }

    Matrix3Xf norMat(){
        return vec2mat(nor);
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
