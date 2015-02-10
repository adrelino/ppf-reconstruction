#include "PointCloud.h"
#include "LoadingSaving.h"

#include <algorithm>    // std::remove_if


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

PointCloud::PointCloud(const string &filename, Matrix3f &K, string maskname,bool showDepthMap){
    cout<<" PointCloud fromDepthMap constructor"<<endl;

    LoadingSaving::loadPointCloudFromDepthMap(filename,K,pts,maskname,showDepthMap);
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
        if(npts<Params::getInstance()->minPtsPerVoxel) continue;

        pts2.push_back(PointCloudManipulation::getCentroid(it.second));
        nor2.push_back(PointCloudManipulation::getNormal(it.second));
        //i++;
    }

    pts=pts2;
    nor=nor2;

    cout<<"DownSampled "<<nOrig<<"->"<<pts.size()<< " pts with voxelSize:"<<voxelSize<<" minPtsPerVoxel:"<<Params::getInstance()->minPtsPerVoxel<<endl;
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
    Isometry3f& P1 = pose;
    for (int j = 0; j < (*frames).size(); ++j) {
        if(i==j) continue;
        Isometry3f& P2 = (*frames)[j]->pose;

//        Quaternionf rot1(P1.linear());

  //      Quaternionf rot2(P2.linear());


        //Translation
        float diff_tra=(P1.translation()-P2.translation()).norm();

        neighboursDescr="diff_tra: ";
        if (diff_tra<tra_thresh) {
            //cout<<i<<" similar to "<<j<<endl;
            neighbours.push_back(make_pair(j,diff_tra));
        }
    }
}

#include <iomanip>

bool myfunction1 (std::pair<int,float> a, std::pair<int,float> b) { return (a.second<b.second); }

void PointCloud::computePoseNeighboursKnn(vector< shared_ptr<PointCloud> >* frames, int i, int k,float cutoff){
    neighbours.clear();
    Isometry3f& P1 = pose;
    for (int j = 0; j < (*frames).size(); ++j){
        if(i==j) continue;
        Isometry3f& P2 = (*frames)[j]->pose;



        //Translation
        float diff_tra=(P1.translation()-P2.translation()).norm();

        neighboursDescr="diff_tra: ";

        if (diff_tra<cutoff){// && meanDist < mean_nn_thresh){

            //stringstream ss;
            //ss<<setprecision(4)<<"cloud neighbours overlap: "<<overlapValue*100<<"% cutoff: "<<cutoff<<" meanDist: "<<meanDist ;

            neighbours.push_back(make_pair(j,diff_tra));
        }
    }

    if(neighbours.size()<k){
        std::sort(neighbours.begin(),neighbours.end(),myfunction1);
    }else{
        std::partial_sort (neighbours.begin(), neighbours.begin()+k, neighbours.end(),myfunction1);
        neighbours.resize(k);
    }

    //auto end = std::remove_if(neighbours.begin(), neighbours.end(), [&](std::pair<int,float> a) { return !frames->at(a.first)->fixed; } );
    //cout<<"size before"<<neighbours.size();
    //neighbours.erase(end, neighbours.end());
    //cout<<"  after"<<neighbours.size()<<endl;


}

void PointCloud::computeCloudNeighbours(vector< shared_ptr<PointCloud> >* frames, int i, float overlap, float cutoff, float mean_nn_thresh){
    neighbours.clear();
    for (int j = 0; j < (*frames).size(); ++j) {
        if(i==j) continue;
        auto other = (*frames)[j];
        if(!other->fixed) continue;
        vector<Vector3f> src,dst,nor,norSrc;
        float meanDist = PointCloudManipulation::getClosesPoints(*this,*other,src,dst,cutoff,true,nor,norSrc); //cutoffDist ==? radius search

        float overlapValue = dst.size()*1.0f / pts.size();

        if (overlapValue>overlap){// && meanDist < mean_nn_thresh){

            //stringstream ss;

            //ss<<setprecision(4)<<"cloud neighbours overlap: ";//<<overlapValue*100<<"% cutoff: "<<cutoff<<" meanDist: "<<meanDist ;

            neighboursDescr="cloud neighbours overlap: ";

            neighbours.push_back(std::make_pair(j,overlapValue));
        }
    }
}

bool myfunction (std::pair<int,float> a, std::pair<int,float> b) { return (a.second>b.second); }

void PointCloud::computeCloudNeighboursKnn(vector< shared_ptr<PointCloud> >* frames, int i, int k,float cutoff){
    neighbours.clear();
    int jMax=(*frames).size();
    //cout<<"frames: "<<jMax<<endl;
    for (int j = 0; j < (*frames).size(); ++j) {
        if(i==j) continue;
        auto other = (*frames)[j];
        //if(!other->fixed) continue;

        vector<Vector3f> src,dst,nor,norSrc;
        float meanDist = PointCloudManipulation::getClosesPoints(*this,*other,src,dst,cutoff,true,nor,norSrc); //cutoffDist ==? radius search

        float overlapValue = dst.size()*1.0f / pts.size();

        neighboursDescr="cloud neighbours overlap Knn: ";

        //if (overlapValue>overlap){// && meanDist < mean_nn_thresh){

            //stringstream ss;
            //ss<<setprecision(4)<<"cloud neighbours overlap: "<<overlapValue*100<<"% cutoff: "<<cutoff<<" meanDist: "<<meanDist ;

            neighbours.push_back(make_pair(j,overlapValue));
        //}
    }

    if(neighbours.size()<k){
        std::sort(neighbours.begin(),neighbours.end(),myfunction);
    }else{
        std::partial_sort (neighbours.begin(), neighbours.begin()+k, neighbours.end(),myfunction);
        neighbours.resize(k);
    }

    auto end = std::remove_if(neighbours.begin(), neighbours.end(), [&](std::pair<int,float> a) { return !frames->at(a.first)->fixed; } );
    //cout<<"size before"<<neighbours.size();
    neighbours.erase(end, neighbours.end());
    //cout<<"  after"<<neighbours.size()<<endl;


}

float PointCloud::computeClosestPointsToNeighboursStacked(vector< shared_ptr<PointCloud> >* frames, float thresh){
    float accumICPErr = 0;
    src.clear();
    dst.clear();
    dstNor.clear();
    srcNor.clear();

    for(int j=0; j<neighbours.size(); j++){
        int idxDstCloud=neighbours[j].first;
        PointCloud& dstCloud = *(*frames)[idxDstCloud].get();
        accumICPErr += PointCloudManipulation::getClosesPoints(*this,dstCloud,src,dst,thresh,true,dstNor,srcNor);
    }

    return accumICPErr;
}


float PointCloud::computeClosestPointsToNeighbours(vector< shared_ptr<PointCloud> >* frames, float thresh){
    float accumICPErr = 0;
    src.clear();
    dst.clear();
    dstNor.clear();
    srcNor.clear();

    vector<Vector3f> preTras(neighbours.size());
    vector<Matrix3f> preInvRots(neighbours.size());


    for(int j=0; j<neighbours.size(); j++){
        int idxDstCloud=neighbours[j].first;
        PointCloud& dstCloud = *(*frames)[idxDstCloud].get();
        preTras[j] = Vector3f(dstCloud.pose.translation());
        preInvRots[j] = dstCloud.pose.linear().inverse();
    }

    PointCloud& srcCloud = *this;


    for (int i = 0; i < srcCloud.pts.size(); ++i) {
        Vector3f& srcPtOrig = srcCloud.pts[i];
        Vector3f srcPtInGlobalFrame = srcCloud.pose*srcPtOrig;

        float diffMin=999999;
        size_t idxMin;
        int idxDstCloudMin;

        for(int j=0; j<neighbours.size(); j++){
            int idxDstCloud=neighbours[j].first;
            PointCloud& dstCloud = *(*frames)[idxDstCloud].get();

            Vector3f srcPtinDstFrame =  preInvRots[j] * (srcPtInGlobalFrame-preTras[j]);
            size_t idx;
            float diff = sqrtf(dstCloud.getClosestPoint(srcPtinDstFrame,idx));
            if(diff<diffMin){
                diffMin=diff;
                idxMin=idx;
                idxDstCloudMin=idxDstCloud;
            }
        }

        PointCloud& dstCloud = *(*frames)[idxDstCloudMin].get();


        Vector3f dstPtInGlobalFrame = dstCloud.pose*dstCloud.pts[idxMin];

        if(diffMin<thresh){ //get rid ouf outlier correspondences
            //corresp.push_back(idxMin);
            accumICPErr += diffMin;//(dstPtBest-srcPt).norm();
            src.push_back(srcPtInGlobalFrame);
            dst.push_back(dstPtInGlobalFrame);
            if(dstCloud.nor.size()>0) dstNor.push_back(dstCloud.pose.linear()*dstCloud.nor[idxMin]);
            if(srcCloud.nor.size()>0) srcNor.push_back(srcCloud.pose.linear()*srcCloud.nor[idxMin]);

        }
    }

    return accumICPErr;
}
