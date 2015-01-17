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

    //mimic depth2cloudAndNormals.m matlab file

    constexpr static float nan = numeric_limits<float>::quiet_NaN();


    static void cloud2normals(const cv::Mat &cloud, cv::Mat &normals, const cv::Mat &mask, vector<Vector3f> &nor)
    {
         const int n=5;
         normals = cv::Mat(cloud.size(),CV_32FC3, cv::Scalar(nan,nan,nan) );
         Matrix3f M;
         for(int r=2*n;r<cloud.rows-2*n-1;++r)
             for(int c=2*n;c<cloud.cols-2*n-1;++c)
             {
                 if(!mask.at<bool>(r,c)) continue;

                 const Vector3f &pt = cloud.at<Vector3f>(r,c);
                 if(pt(2)==0) continue;

                 const float thresh = 0.08f*pt(2);
                 M.setZero();
                 for (int i=-n; i <= n; i+=n)
                     for (int j=-n; j <= n; j+=n)
                     {
                         Vector3f curr = cloud.at<Vector3f>(r+i,c+j)-pt;
                         if (fabs(curr(2)) > thresh) continue;
                         M += curr*curr.transpose();
                     }

                 Vector3f &no = normals.at<Vector3f>(r,c);
                 EigenSolver<Matrix3f> es(M);
                 int i; es.eigenvalues().real().minCoeff(&i);
                 no = es.eigenvectors().col(i).real();

                 // JacobiSVD<Matrix3f> svd(M, ComputeFullU | ComputeFullV);
                 // no = svd.matrixV().col(2);
                 if (no(2) > 0) no = -no;

                 nor.push_back(no);

             }
    }

    static PointCloud fromDepthImage(cv::Mat depth, cv::Mat mask, float fx, float fy, float cx, float cy){
        int m=depth.rows; //Y
        int n=depth.cols; //X

        vector<Eigen::Vector3f> pts;
        //cv::Mat ptsOrdered = cv::Mat(depth.size(),CV_32FC3, cv::Scalar(nan,nan,nan) );

        int x=n,y=m,xM=0,yM=0;

        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                if(mask.at<bool>(i,j)){
                    float Z = depth.at<float>(i,j);
                    float X = (j-cx)*Z*(1/fx);
                    float Y = (i-cy)*Z*(1/fy);
                    Vector3f pt(X,Y,Z);
                    pts.push_back(pt);
                    //ptsOrdered.at<Vector3f>(i,j)=pt;

                    if(j<x) x=j;if(j>xM) xM=j;if(i<y) y=i;if(i>yM) yM=i;
                }
            }
        }

       vector<Eigen::Vector3f> nor;
       //cv::Mat norOrdered; //= cv::Mat(depth.size(),CV_32FC3, cv::Scalar(nan,nan,nan) );

       //cloud2normals(ptsOrdered,norOrdered,mask,nor);

        PointCloud C;
        C.pts=pts;
        C.nor=nor;

        return C;
    }

    static PointCloud fromDepthImage(cv::Mat depth, cv::Mat mask, Eigen::Matrix3f K){
        float fx = K(0,0);
        float fy = K(1,1);
        float cx = K(0,2);
        float cy = K(1,2);
        return fromDepthImage(depth,mask,fx,fy,cx,cy);
    }

    static PointCloud fromDepthImage(cv::Mat depth, cv::Mat mask){

        int m=depth.rows; //Y
        int n=depth.cols; //X

        float fx = 542;
        float fy = 540.5;
        float cx = n/2;
        float cy = m/2;
        return fromDepthImage(depth,mask,fx,fy,cx,cy);
    }

    /*void normalEstimationPAMIHInt(){
        //        [M,N]=ind2sub(size(depth),find(depth>0));
        //        x=min(N);y=min(M);xM=max(N);yM=max(M);

        //        %begin normal estimation
        //        normals = zeros(m,n,3);
               int win=5;
         for(int i=y+win; i<yM-win; i++){
             for(int j=x+win; i<xM-win; i++){
                 if(!mask.at<bool>(i,j)) continue;

                 Vector4f A = Vector4f::Zero();
                 Vector2f b = Vector2f::Zero();

                 float val = depth.at<float>(i,j);

                 for(int k=-win; k<win; k++){
                     for(int l=-win; l<win; l++){
                         float diff = val-depth.at<float>(i+k,j+l);
                         if (abs(diff) < 0.005){
                             A(0) = A(0) + k*k;
                             A(1) = A(1) + k*l;
                             A(3) = A(3) + l*l;
                             b(0) = b(0) + k*diff;
                             b(1) = b(1) + l*diff;
                         }
                     }
                 }

                 float det =    A(0)*A(3) - A(1)*A(1);
                 float ddx =  ( A(3)*b(0) - A(1)*b(1));
                 float ddy =  (-A(1)*b(0) + A(0)*b(1));
                 Vector3f normal(fx*ddx,fy*ddy,-det*val);
                 nor.push_back(normal.normalized());
             }
         }
    }*/

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

    //if 0 entries we take default Visualizer color, if 1 entry all points share color, else per point color
    std::vector<RowVector3f> pts_color;
    std::vector<RowVector3f> nor_color;

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

    void translateToCentroid(){
        Matrix3Xf m = vec2mat(pts);
        Vector3f mean1=m.rowwise().mean();

        pts=mat2vec(m.colwise()-mean1);
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
