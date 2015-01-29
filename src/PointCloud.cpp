#include "PointCloud.h"
#include "LoadingSaving.h"

using namespace Eigen;
using namespace std;

PointCloud::PointCloud(){
    cout<<" PointCloud default constructor"<<endl;

}

PointCloud::PointCloud(const PointCloud &other){
    cout<<" PointCloud copy constructor"<<endl;
    pts=other.pts;
    nor=other.nor;
}

PointCloud::PointCloud(const string &filename, Matrix3f &K){
    cout<<" PointCloud fromDepthMap constructor"<<endl;

    LoadingSaving::loadPointCloudFromDepthMap(filename,K,pts,false);
}

PointCloud::~PointCloud(){
    cout<<" PointCloud destructor"<<endl;
    if(indexComputed) delete indexPtr;
}

std::vector<PPF> PointCloud::getPPFFeatures(){
    if(featuresComputed){
        return features;
    }else{
        int Nm=pts.size();
        //cout<<"PointPairFeatures::computePPFFeatures from "<<Nm<<" pts"<<endl;

        int numFeatures = Nm * (Nm-1); //exclude diagonal

        features = std::vector<PPF>(numFeatures);

        int k=0;

        for (int i=0; i<Nm; i++) {
            for (int j=0; j<Nm; j++) {
                if(i==j) continue;

                //PPF ppf(m,i,j);

                features[k++]=PPF(pts,nor,i,j); //i*Nm + j

                //map.push_back(ppf);
            }
        }

        std::sort(features.begin(),features.end());
        featuresComputed=true;

        return features;
    }
}

Matrix3Xf PointCloud::ptsMat(){
    return vec2mat(pts);
}

Matrix3Xf PointCloud::norMat(){
    return vec2mat(nor);
}

vector<Vector3f> const PointCloud::getPtsInGlobalFrame(){
    return mat2vec(pose * ptsMat());
}

const vector<Vector3f> PointCloud::getNorInGlobalFrame(){
    return mat2vec(pose.linear() * norMat());
}

void PointCloud::setPose(Isometry3f P){
    pose = P;
}

void PointCloud::setPoseGroundTruth(Isometry3f P){
    poseGroundTruth=P;
}

float PointCloud::getPoseError(){
    return err(pose,poseGroundTruth);
}

void PointCloud::downsample(float voxelSize){

    unordered_map<string, vector<Vector3f> > voxels;

    int nOrig=pts.size();

    for (int i=0; i<nOrig; i++) {
        int x=floor(pts[i].x()/voxelSize);
        int y=floor(pts[i].y()/voxelSize);
        int z=floor(pts[i].z()/voxelSize);
        stringstream ss;
        ss<<x<<"|"<<y<<"|"<<z;
        string key=ss.str();
        voxels[key].push_back(pts[i]);
    }

    //int i=0;

    vector<Vector3f> pts2,nor2;

    for (auto it : voxels){
        int npts = it.second.size();
        if(npts<minPtsPerVoxel) continue;

        pts2.push_back(PointCloudManipulation::getCentroid(it.second));
        nor2.push_back(PointCloudManipulation::getNormal(it.second));
        //i++;
    }

    pts=pts2;
    nor=nor2;

    cout<<"DownSampled "<<nOrig<<"->"<<pts.size()<< " pts with voxelSize:"<<voxelSize<<" minPtsPerVoxel:"<<minPtsPerVoxel<<endl;
}


float PointCloud::getClosestPoint(const Vector3f& query_pt, size_t& ret_index){ //query_pt must be in dstFrame
    if(!indexComputed){
        //PointCloud::PointCloud():
        //PointCloud C = *this;
        indexPtr = new my_kd_tree_t(3 /*dim*/, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        indexPtr->buildIndex();

        //indexPtr = &index;
        cout<<"flann: build index"<<endl;

        indexComputed=true;

    }

    //{
        // do a knn search
        const size_t num_results = 1;
        //size_t ret_index;
        float out_dist_sqr;
        nanoflann::KNNResultSet<float> resultSet(num_results);
        resultSet.init(&ret_index, &out_dist_sqr );
        indexPtr->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(32,0.01,false));

        //std::cout << "knnSearch(nn="<<num_results<<"): \n";
        //std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << endl;

        return out_dist_sqr;

        //return pts[ret_index];

   // }
//    {
//        // Unsorted radius search:
//        const float radius = 1;
//        std::vector<std::pair<size_t,float> > indices_dists;
//        nanoflann::RadiusResultSet<float,size_t> resultSet(radius,indices_dists);

//        index.findNeighbors(resultSet, query_pt, nanoflann::SearchParams(32,0,true));

//        // Get worst (furthest) point, without sorting:
//        std::pair<size_t,float> worst_pair = resultSet.worst_item();
//        resultSet.
//        cout << "Worst pair: idx=" << worst_pair.first << " dist=" << worst_pair.second << endl;
//    }
}

void PointCloud::computePoseNeighbours(vector< shared_ptr<PointCloud> >* frames, int i, float tra_thresh, float rot_thresh){
    neighbours.clear();
    for (int j = 0; j < (*frames).size(); ++j) {
        if(i==j) continue;
        Isometry3f& other = (*frames)[j]->pose;
        if (isPoseSimilar(pose,other,rot_thresh,tra_thresh)) {
            //cout<<i<<" similar to "<<j<<endl;
            neighbours.push_back(j);
        }
    }
}

void PointCloud::computeCloudNeighbours(vector< shared_ptr<PointCloud> >* frames, int i, float poinCloudOverlap, float cutoffDist, float nearestNeighbourMeanDist_thresh){

}

float PointCloud::computeClosestPointsToNeighbours(vector< shared_ptr<PointCloud> >* frames, vector<Vector3f>& src, vector<Vector3f>& dst, float thresh, bool useFlann, vector<Vector3f>& nor){
    float accumICPErr = 0;

    for(int i : neighbours){
        accumICPErr += PointCloudManipulation::getClosesPoints(*this,*(*frames)[i].get(),src,dst,ddist,useFlann,nor);
    }

    return accumICPErr;
}
