#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <eigen3/Eigen/Dense>
#include <opencv2/core.hpp>

#include <vector>
#include "PPF.h"
#include "Constants.h"
#include "nanoflann.hpp"
#include "PointCloudManipulation.h"

using namespace Eigen;
using namespace std;

class PointCloud{

public:
    vector<PPF> features;
    bool featuresComputed = false;

    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloud > ,
        PointCloud,
        3 /* dim */
        > my_kd_tree_t;

    my_kd_tree_t* indexPtr;
    bool indexComputed = false;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t /*size*/) const
    {
        const float d0=p1[0]-pts[idx_p2].x();
        const float d1=p1[1]-pts[idx_p2].y();
        const float d2=p1[2]-pts[idx_p2].z();
        return d0*d0+d1*d1+d2*d2;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline float kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim==0) return pts[idx].x();
        else if (dim==1) return pts[idx].y();
        else return pts[idx].z();
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

    PointCloud();
    PointCloud(const PointCloud& other);
    PointCloud(const std::string& filename, Matrix3f& K);

    vector<Vector3f> pts;
    vector<Vector3f> nor;
//    std::vector<float> cur;

    //if 0 entries we take default Visualizer color, if 1 entry all points share color, else per point color
    vector<Vector3f> pts_color;
    vector<Vector3f> nor_color;

    vector<int> neighbours;

    Isometry3f pose = Isometry3f::Identity();
    Isometry3f poseGroundTruth = Isometry3f::Identity();

    constexpr static float nan = numeric_limits<float>::quiet_NaN();

    std::vector<PPF> getPPFFeatures();

    Matrix3Xf ptsMat();
    Matrix3Xf norMat();

    const vector<Vector3f> getPtsInGlobalFrame();
    const vector<Vector3f> getNorInGlobalFrame();

    void setPose(Isometry3f P);
    void setPoseGroundTruth(Isometry3f P);

    float getPoseError();

    float getClosestPoint(const Vector3f& query_pt, size_t& ret_index);

    ~PointCloud();

    void downsample(float voxelSize);

    void computePoseNeighbours(vector< shared_ptr<PointCloud> >* frames, int i, float tra_thresh, float rot_thresh);
    void computeCloudNeighbours(vector< shared_ptr<PointCloud> >* frames, int i, float poinCloudOverlap, float cutoffDist, float nearestNeighbourMeanDist_thresh);


    float computeClosestPointsToNeighbours(vector< shared_ptr<PointCloud> >* frames, vector<Vector3f>& src, vector<Vector3f>& dst, float thresh, bool useFlann, vector<Vector3f>& nor);

};

inline std::ostream& operator<<(std::ostream& os, const PointCloud& v) {
    os<<v.neighbours.size()<<" neighbours: ";
    for(int j : v.neighbours){
        os<<j<<",";
    }
    os<<endl;

    Vector3f tra(v.pose.translation());
    Quaternionf q(v.pose.linear());
    Vector4f rot(q.x(),q.y(),q.z(),q.w());

    os<<"tra: "<<tra.transpose()<<endl;
    os<<"rot: "<<rot.transpose()<<endl;


 // os << "size: " <<  v.pts.size() <<std::endl;
//     << "pts:" << std::endl << vec2mat(v.pts) <<std::endl \
//     << "nor:" << std::endl << vec2mat(v.nor)<< std::endl;
  return os;
}

#endif // POINTCLOUD_H
