#include "PointCloud.h"
#include "LoadingSaving.h"

#include <algorithm>    // std::remove_if


using namespace Eigen;
using namespace std;

PointCloud::PointCloud(){
    cout<<" PointCloud default constructor"<<endl;

}

//PointCloud::PointCloud(const PointCloud &other){
//    cout<<" PointCloud copy constructor"<<endl;
//    pts=other.pts;
//    nor=other.nor;
//    cur=other.cur;
//    curColor=other.curColor;
//}

PointCloud::PointCloud(const string &filename, Matrix3f &K, string maskname,bool showDepthMap){
    cout<<" PointCloud fromDepthMap constructor"<<endl;

    LoadingSaving::loadPointCloudFromDepthMap(filename,K,ptsOrig,maskname,showDepthMap);
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

    int nOrig=ptsOrig.size();

    for (int i=0; i<nOrig; i++) {
        int x=floor(ptsOrig[i].x()/voxelSize);
        int y=floor(ptsOrig[i].y()/voxelSize);
        int z=floor(ptsOrig[i].z()/voxelSize);
        stringstream ss;
        ss<<x<<"|"<<y<<"|"<<z;
        string key=ss.str();
        voxels[key].push_back(ptsOrig[i]);
    }

    pts.clear();
    nor.clear();
    cur.clear();


    for (auto it : voxels){
        int npts = it.second.size();
        if(npts<Params::getInstance()->minPtsPerVoxel || npts<3) continue;

            Vector3f centroid,normal;
            float curvature;

            PointCloudManipulation::pointSetPCA(it.second,centroid,normal,curvature);

            pts.push_back(centroid);
            nor.push_back(normal);
            cur.push_back(curvature);
        //i++;
    }

    centerOfMass = PointCloudManipulation::getCentroid(pts);


    cout<<"DownSampled "<<nOrig<<"->"<<pts.size()<< " pts with voxelSize:"<<voxelSize<<" minPtsPerVoxel:"<<Params::getInstance()->minPtsPerVoxel<<endl;
}

float PointCloud::getClosestPointInGlobalFrameLinear(const Vector3f& query_ptInGlobalFrame, size_t& ret_index){
    float diffMin=99999999;
    for (int i = 0; i < pts.size(); ++i) {
        Vector3f dstPtinGlobal = pose*pts[i];
        float diff = (query_ptInGlobalFrame-dstPtinGlobal).norm();
        if(diff<diffMin){
            diffMin=diff;
            ret_index=i;
        }
    }
    return diffMin*diffMin;
}

vector<Vector3f> PointCloud::getCurvColors(){
    if(curColor.size()>0) return curColor;
    auto minMaxIdx = std::minmax_element(cur.begin(),cur.end());
    float min = cur[minMaxIdx.first-cur.begin()];
    float max = cur[minMaxIdx.second-cur.begin()];
    cout<<"Curv min:"<<min<<"  max:"<<max<<endl;

//    min=0.01f;
//    max=0.05f;
    for(float val : cur){
       //Vector3f color1 = Params::getInstance()->colorJet(val,min,max);
       Vector3f color2 = Colormap::Jet((val-min)/(max-min));
       curColor.push_back(color2);
    }
    return curColor;
}


float PointCloud::getClosestPoint(const Vector3f& query_pt, size_t& ret_index){ //query_pt must be in dstFrame
    if(!indexComputed){
        //PointCloud::PointCloud():
        //PointCloud C = *this;
        indexPtr = new my_kd_tree_t(3 /*dim*/, *this, nanoflann::KDTreeSingleIndexAdaptorParams(1 /* max leaf */));
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
        indexPtr->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(32,0,false));

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
            neighbours.push_back({j,diff_tra});
        }
    }
}

#include <iomanip>

bool myfunction1 (OutgoingEdge a, OutgoingEdge b) { return (a.weight<b.weight); }

void PointCloud::computePoseNeighboursKnn(vector< shared_ptr<PointCloud> >* frames, int i, int k,float cutoff){
    int nsize = neighbours.size();
    cout<<"nsize: "<<nsize<<endl;


    OutgoingEdge pairWiseNeighbour;
    if (nsize>0) {
        pairWiseNeighbour= neighbours[0];
    }

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

            neighbours.push_back({j,diff_tra});
        }
    }

    if(neighbours.size()<k){
        std::sort(neighbours.begin(),neighbours.end(),myfunction1);
    }else{
        std::partial_sort (neighbours.begin(), neighbours.begin()+k, neighbours.end(),myfunction1);
        neighbours.resize(k);
    }

    cout<<"neighbours size"<<neighbours.size()<<endl;

    if(nsize){
        if ( std::any_of(neighbours.begin(), neighbours.end(), [=](OutgoingEdge e){return pairWiseNeighbour.neighbourIdx==e.neighbourIdx ;}) ){
            cout<<"neighbour already added"<<endl;
        }else{
            cout<<"adding pairwise neighbour"<<endl;
            pairWiseNeighbour.weight=0;
            neighbours.push_back(pairWiseNeighbour);
        }
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

            neighbours.push_back({j,overlapValue});
        }
    }
}

bool myfunction (OutgoingEdge a, OutgoingEdge b) { return (a.weight>b.weight); }

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

            neighbours.push_back({j,overlapValue});
        //}
    }

    if(neighbours.size()<k){
        std::sort(neighbours.begin(),neighbours.end(),myfunction);
    }else{
        std::partial_sort (neighbours.begin(), neighbours.begin()+k, neighbours.end(),myfunction);
        neighbours.resize(k);
    }

    auto end = std::remove_if(neighbours.begin(), neighbours.end(), [&](OutgoingEdge a) { return !frames->at(a.neighbourIdx)->fixed; } );
    //cout<<"size before"<<neighbours.size();
    neighbours.erase(end, neighbours.end());
    //cout<<"  after"<<neighbours.size()<<endl;


}

float PointCloud::computeClosestPointsToNeighboursStacked(vector< shared_ptr<PointCloud> >* frames, float thresh){
    float accumICPErr = 0;
//    src.clear();
//    dst.clear();
//    dstNor.clear();
//    srcNor.clear();

//    for(int j=0; j<neighbours.size(); j++){
//        int idxDstCloud=neighbours[j].neighbourIdx;
//        PointCloud& dstCloud = *(*frames)[idxDstCloud].get();
//        accumICPErr += PointCloudManipulation::getClosesPoints(*this,dstCloud,src,dst,thresh,true,dstNor,srcNor);
//    }

    return accumICPErr;
}

float PointCloud::computeClosestPointsToNeighboursRelative(vector< shared_ptr<PointCloud> >* frames, float thresh){

    accumError = 0;

    PointCloud& srcCloud = *this;

    int N = srcCloud.neighbours.size();
    assert(N==1);

    //for (int j = 0; j < srcCloud.neighbours.size(); ++j) {
        OutgoingEdge& edge = srcCloud.neighbours[0];

        int dst_id=edge.neighbourIdx;
        edge.correspondances.clear();

        PointCloud& dstCloud = *frames->at(dst_id);

        Isometry3f dstToSrc = edge.P_relative;

        float error=0;

        for (int k = 0; k < srcCloud.pts.size(); ++k) {
            Vector3f& srcPtOrig = srcCloud.pts[k];
            size_t idxMin;

            Vector3f srcPtinDstFrame = dstToSrc * srcPtOrig;
            float pointDistSquared = dstCloud.getClosestPoint(srcPtinDstFrame,idxMin);

            if(pointDistSquared<thresh*thresh){
                error += pointDistSquared;
                edge.correspondances.push_back(make_pair(k,idxMin));
            }
        }

        edge.weight=edge.correspondances.size();//error;
        accumError +=error;
    //}

    return accumError;
}


float PointCloud::computeClosestPointsToNeighbours(vector< shared_ptr<PointCloud> >* frames, float thresh){

    accumError = 0;

    PointCloud& srcCloud = *this;

    for (int j = 0; j < srcCloud.neighbours.size(); ++j) {

        int dst_id=srcCloud.neighbours[j].neighbourIdx;
        srcCloud.neighbours[j].correspondances.clear();

        PointCloud& dstCloud = *frames->at(dst_id);

        Vector3f preTra = Vector3f(dstCloud.pose.translation());
        auto preInvRot = dstCloud.pose.linear().inverse();

        float error=0;

        for (int k = 0; k < srcCloud.pts.size(); ++k) {
            Vector3f& srcPtOrig = srcCloud.pts[k];
            Vector3f srcPtInGlobalFrame = srcCloud.pose*srcPtOrig;


            size_t idxMin;

            Vector3f srcPtinDstFrame = preInvRot * (srcPtInGlobalFrame-preTra);
            float pointDistSquared = dstCloud.getClosestPoint(srcPtinDstFrame,idxMin);

            //Vector3f srcPtinDstFrame = preInv*srcPtInGlobalFrame;
            //pointDistSquared = dstCloud.getClosestPointInGlobalFrameLinear(srcPtInGlobalFrame,idxMin);

            if(pointDistSquared<thresh*thresh){
                error += pointDistSquared;
                srcCloud.neighbours[j].correspondances.push_back(make_pair(k,idxMin));
            }
        }

        srcCloud.neighbours[j].weight=srcCloud.neighbours[j].correspondances.size();//error;
        accumError +=error;
    }

    return accumError;
}

float PointCloud::recalcError(vector< shared_ptr<PointCloud> >* frames){

    PointCloud& srcCloud = *this;

    accumError = 0;

    float numbEdges =0;

    for (int j = 0; j < srcCloud.neighbours.size(); ++j) {

        int dst_id=srcCloud.neighbours[j].neighbourIdx;

        PointCloud& dstCloud = *frames->at(dst_id);

        float finalchi2=0;
        for (auto corr : srcCloud.neighbours[j].correspondances) {
            Vector3f diff = srcCloud.pose*srcCloud.pts[corr.first]-dstCloud.pose*dstCloud.pts[corr.second];
            finalchi2 += diff.dot(diff);
            numbEdges +=1;
        }

        //srcCloud.neighbours[j].weight=finalchi2;
        accumError +=finalchi2;
    }

    return numbEdges;
}

// #include <g2o/core/sparse_optimizer.h>
// #include <g2o/core/block_solver.h>
// #include <g2o/core/solver.h>
// #include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>
// #include <g2o/solvers/csparse/linear_solver_csparse.h>

// #include <g2o/core/optimization_algorithm_gauss_newton.h>
// //#include <g2o/core/optimization_algorithm_dogleg.h>

// #include <g2o/types/icp/types_icp.h>

//#include "types_icp.h"

using namespace Eigen;
using namespace std;
// using namespace g2o;
//using namespace g2o2;

float eps = 0.0001f;

bool PointCloud::alignToFirstNeighbourWithICP(vector< shared_ptr<PointCloud> >* frames, bool pointToPlane, bool useRelativeEdge){
    Isometry3f P_incemental;

    assert(neighbours.size()==1);

    OutgoingEdge& dstEdge = neighbours[0];

    PointCloud& dstCloud = *frames->at(dstEdge.neighbourIdx);

    // bool normalICP = true;

    Isometry3f P_relative = dstEdge.P_relative;

    // if(normalICP){
        vector<Vector3f> src,dst,dstNor;
        for (auto corr : dstEdge.correspondances) {
            if(useRelativeEdge){
            src.push_back(P_relative*pts[corr.first]);
            dst.push_back(dstCloud.pts[corr.second]);
            dstNor.push_back(dstCloud.nor[corr.second]);
            }else{
            src.push_back(pose*pts[corr.first]);
            dst.push_back(dstCloud.pose*dstCloud.pts[corr.second]);
            dstNor.push_back(dstCloud.pose.linear()*dstCloud.nor[corr.second]);
            }
        }



        if(pointToPlane){
            P_incemental = ICP::pointToPlane(src,dst,dstNor);//*src,*dst,*nor); //point to plane
        }else{
            P_incemental = ICP::pointToPoint(src,dst); //point to point
        }



        if(isPoseCloseToIdentity(P_incemental,eps)){
            return true;
        }

        if(useRelativeEdge){
            dstEdge.P_relative =  P_incemental * P_relative;
            pose = dstCloud.pose * dstEdge.P_relative;
        }else{
           pose = P_incemental * pose;
        }

    // }else{
    //     SparseOptimizer optimizer;
    //     optimizer.setVerbose(true);
    //     BlockSolverX::LinearSolverType * linearSolver = new LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    //     BlockSolverX * solver_ptr = new BlockSolverX(linearSolver);
    //     g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //     optimizer.setAlgorithm(solver);

    //     {
    //         VertexSE3 *vc = new VertexSE3();
    //         vc->setEstimate(dstCloud.pose.cast<double>());
    //         dstCloud.fixed=true;
    //         vc->setFixed(true);
    //         vc->setId(0);
    //         optimizer.addVertex(vc);
    //     }

    //     {
    //         VertexSE3 *vc = new VertexSE3();
    //         vc->setEstimate(pose.cast<double>());
    //         fixed=false;
    //         vc->setFixed(false);
    //         vc->setId(1);
    //         optimizer.addVertex(vc);
    //     }

    //     // get two poses
    //     VertexSE3* vp0 =
    //       dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second); //dstCloud fixed
    //     VertexSE3* vp1 =
    //       dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second); //srcCloud ==this

    //     for (auto corr : dstEdge.correspondances) {

    //         Edge_V_V_GICP * e = new Edge_V_V_GICP();

    //         e->setVertex(0, vp0);      // first viewpoint
    //         e->setVertex(1, vp1);      // second viewpoint

    //         EdgeGICP meas;
    //         meas.pos0 = dstCloud.pts[corr.second].cast<double>();
    //         meas.pos1 = pts[corr.first].cast<double>();
    //         meas.normal0 = dstCloud.nor[corr.second].cast<double>();
    //         meas.normal1 = nor[corr.first].cast<double>();

    //         e->setMeasurement(meas);

    //         if(pointToPlane){

    //         //meas = e->measurement();
    //         // use this for point-plane
    //         e->information() = meas.prec0(0.01);
    //         }else{

    //         // use this for point-point
    //         e->information().setIdentity();
    //         }

    //         optimizer.addEdge(e);
    //     }

    //     int numbEdges = recalcError(frames);

    //     cerr<<"errrorrrr: "<<accumError<<"  numbEdges: "<<numbEdges <<" pointToPlane"<<pointToPlane<<endl;


    //     optimizer.initializeOptimization();
    //     optimizer.computeActiveErrors();
    //     cerr << "Initial activeChi2 = " << FIXED(optimizer.activeChi2()) << endl;
    //     cerr << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

    //     optimizer.setVerbose(true);

    //     solver->setUserLambdaInit(50000);
    //     optimizer.optimize(300);

    //     Isometry3d test = dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second)->estimate();

    //     pose = test.cast<float>();
    // }


    return false;
}
//#include "Visualize.h"
void PointCloud::updateChildrenAbsolutePoses(vector<shared_ptr<PointCloud> >& frames, int myIdx){
    //cout<<"myIdx: "<<myIdx<<endl;
    for (int childIdx : children) {
        //cout<<"childIdx: "<<childIdx<<endl;
        assert(frames[childIdx]->neighbours.size()==1);
        OutgoingEdge& edge = frames[childIdx]->neighbours[0];
        //cout<<"neighbourIdx: "<<edge.neighbourIdx<<endl;

        assert(edge.neighbourIdx==myIdx);

        frames[childIdx]->pose = pose * edge.P_relative;
        //Visualize::spin();
        frames[childIdx]->updateChildrenAbsolutePoses(frames,childIdx);
    }
}
