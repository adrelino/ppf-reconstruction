//
//  PointCloudManipulation.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "PointCloudManipulation.h"
#include "Visualize.h"
#include "PointPairFeatures.h"

namespace ICP {

//K called correlation matrix in eggert_comparison_mva97, is actually a covariance matrix multiplied by N
//http://graphics.stanford.edu/~smr/ICP/comparison/eggert_comparison_mva97.pdf
Isometry3f pointToPoint(vector<Vector3f>&src,vector<Vector3f>&dst){
    int N = src.size(); assert(N==dst.size());
    Map<Matrix3Xf> ps(&src[0].x(),3,N); //maps vector<Vector3f>
    Map<Matrix3Xf> qs(&dst[0].x(),3,N); //to Matrix3Nf columnwise
    Vector3f p_dash = ps.rowwise().mean();
    Vector3f q_dash = qs.rowwise().mean();
    Matrix3Xf ps_centered = ps.colwise() - p_dash;
    Matrix3Xf qs_centered = qs.colwise() - q_dash;
    Matrix3f K = qs_centered * ps_centered.transpose();
    JacobiSVD<Matrix3f> svd(K, ComputeFullU | ComputeFullV);
    
    //this flipping scheme (to make sure we have rotation, not mirroring) is wrong / not always correct!!!!
    /*Matrix3f R = svd.matrixU()*svd.matrixV().transpose();
    if(R.determinant()<0){
        R.col(2) *= -1;
    }*/

    //this is the correct way!!!
    Matrix3f S = Matrix3f::Identity();
    if(svd.matrixU().determinant() * svd.matrixV().transpose().determinant()<0){
        S(2,2)= -1;
    }
    
    Matrix3f R = svd.matrixU()*S*svd.matrixV().transpose();

    
    Isometry3f T = Isometry3f::Identity();
    T.linear() = R;
    T.translation() = q_dash - R*p_dash; return T;
}

// https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
// http://www.cs.princeton.edu/~smr/papers/icpstability.pdf
Isometry3f pointToPlane(vector<Vector3f> &src,vector<Vector3f> &dst,vector<Vector3f> &nor){
    assert(src.size()==dst.size() && src.size()==nor.size());
    Matrix<float,6,6> C; C.setZero();
    Matrix<float,6,1> d; d.setZero();

    for(uint i=0;i<src.size();++i){
        Vector3f cro = src[i].cross(nor[i]);
        C.block<3,3>(0,0) += cro*cro.transpose();
        C.block<3,3>(0,3) += nor[i]*cro.transpose();
        C.block<3,3>(3,3) += nor[i]*nor[i].transpose();
        float sum = (src[i]-dst[i]).dot(nor[i]);
        d.head(3) -= cro*sum;
        d.tail(3) -= nor[i]*sum;
    }
    C.block<3,3>(3,0) = C.block<3,3>(0,3);

    Matrix<float,6,1> x = C.ldlt().solve(d);
    Isometry3f T = Isometry3f::Identity();
    T.linear() = (   AngleAxisf(x(0), Vector3f::UnitX())
                   * AngleAxisf(x(1), Vector3f::UnitY())
                   * AngleAxisf(x(2), Vector3f::UnitZ())
                 ).toRotationMatrix();
    T.translation() = x.block(3,0,3,1);
    return T;
}

// Point to Point with Scale
// http://www5.informatik.uni-erlangen.de/Forschung/Publikationen/2005/Zinsser05-PSR.pdf
Isometry3f computeStepPointToPointWithScale(vector<Vector3f> &src,vector<Vector3f> &dst,bool withScale)
{
    assert(src.size()==dst.size());
    Vector3f a = PointCloudManipulation::getCentroid(src);
    Vector3f b = PointCloudManipulation::getCentroid(dst);

    Matrix3f K = Matrix3f::Zero();
    for (uint i=0; i < src.size();i++)
        K += (b-dst[i])*(a-src[i]).transpose();


    JacobiSVD<Matrix3f> svd(K, ComputeFullU | ComputeFullV);
    Matrix3f R = svd.matrixU()*svd.matrixV().transpose();
    if(R.determinant()<0) R.col(2) *= -1;

    if (withScale)
    {
        float s_up=0,s_down=0;
        for (uint i=0; i < src.size();i++)
        {
            Vector3f b_tilde = (b-dst[i]);
            Vector3f a_tilde = R*(a-src[i]);
            s_up += b_tilde.dot(a_tilde);
            s_down += a_tilde.dot(a_tilde);
        }
        R *= s_up/s_down;
    }

    Isometry3f transform = Isometry3f::Identity();
    transform.linear() = R;
    transform.translation() = b - R*a;
    return transform;
}

} //end namespace ICP

namespace PointCloudManipulation{
CPUTimer timer = CPUTimer();

void pointSetPCA(const vector<Vector3f>& pts, Vector3f& centroid, Vector3f& normal, float& curvature){

    assert(pts.size()>=3); //otherwise normals are undetermined
    Map<const Matrix3Xf> P(&pts[0].x(),3,pts.size());

    centroid = P.rowwise().mean();
    MatrixXf centered = P.colwise() - centroid;
    Matrix3f cov = centered * centered.transpose();

    //eigvecs sorted in increasing order of eigvals
    SelfAdjointEigenSolver<Matrix3f> eig(cov);
    normal = eig.eigenvectors().col(0); //is already normalized
    if (normal(2) > 0) normal = -normal; //flip towards camera
    Vector3f eigVals = eig.eigenvalues();
    curvature = eigVals(0) / eigVals.sum();
}

Isometry3d pointToPointD(vector<Vector3d>&src,vector<Vector3d>&dst){
    int N = src.size(); assert(N==dst.size());
    Map<Matrix3Xd> ps(&src[0].x(),3,N); //maps vector<Vector3f>
    Map<Matrix3Xd> qs(&dst[0].x(),3,N); //to Matrix3Nf columnwise
    Vector3d p_dash = ps.rowwise().mean();
    Vector3d q_dash = qs.rowwise().mean();
    Matrix3Xd ps_centered = ps.colwise() - p_dash;
    Matrix3Xd qs_centered = qs.colwise() - q_dash;
    Matrix3d K = qs_centered * ps_centered.transpose();
    JacobiSVD<Matrix3d> svd(K, ComputeFullU | ComputeFullV);

    //U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
    Matrix3d S = Matrix3d::Identity();
    if(svd.matrixU().determinant() * svd.matrixV().transpose().determinant()<0){
        S(2,2)= -1;
    }
    //rot = U*S*Vh
    Matrix3d R = svd.matrixU()*S*svd.matrixV().transpose();


//    Matrix3d R = svd.matrixU()*svd.matrixV().transpose();
//    if(R.determinant()<0){
//        R.col(2) *= -1;
//    }

    Isometry3d T = Isometry3d::Identity();
    T.linear() = R;
    T.translation() = q_dash - R*p_dash; return T;
}

Isometry3d leastSquaresEstimatedTrajectoryOntoGroundTruth(std::vector< std::shared_ptr<PointCloud> >& frames){
    std::vector<Vector3d> src,dst;
    for (int i = 0; i < frames.size(); ++i) {
        PointCloud& currentFrame = *frames[i];
        //if(currentFrame.neighbours.size()>0 || currentFrame.fixed){
            src.push_back(currentFrame.pose.translation().cast<double>());
            dst.push_back(currentFrame.poseGroundTruth.translation().cast<double>());
        //}
    }
    return pointToPointD(src,dst);
}

vector<double> ateVector(std::vector< std::shared_ptr<PointCloud> >& frames, Isometry3d S){
    int N = frames.size();
    vector<double> ateVec(N);

    for (int i = 0; i < N; ++i) {
        PointCloud& currentFrame = *frames[i];
        Isometry3d Q_i = currentFrame.poseGroundTruth.cast<double>();
        Isometry3d P_i = currentFrame.pose.cast<double>();

        Isometry3d F_i = Q_i.inverse() * S * P_i;
        Vector3d t = F_i.translation();

        ateVec[i] = t.norm();
    }

    return ateVec;
}

vector<double> rpeVector(std::vector< std::shared_ptr<PointCloud> >& frames, int delta){
    int N = frames.size();
    int m = N - delta;
    assert(delta>0 && delta<=N);
    vector<double> rpeVec(m);

    for (int i = 0; i < m; ++i) {
        PointCloud& currentFrame = *frames[i];
        Isometry3d Q_i = currentFrame.poseGroundTruth.cast<double>();
        Isometry3d P_i = currentFrame.pose.cast<double>();

        Isometry3d Q_i_plus_delta = frames[i+delta]->poseGroundTruth.cast<double>();
        Isometry3d P_i_plus_delta = frames[i+delta]->pose.cast<double>();

        Isometry3d E = (Q_i.inverse() * Q_i_plus_delta ).inverse() * (P_i.inverse() * P_i_plus_delta);

        Vector3d t = E.translation();
        rpeVec[i] = t.norm();
    }

    return rpeVec;
}



float registrationErrorTra(vector< std::shared_ptr<PointCloud> >& frames){

    float tra_mean=0;

    for(std::shared_ptr<PointCloud> it : frames){
        Vector3f    tra1 = it->pose.translation();

        Vector3f    tra2 = it->poseGroundTruth.translation();

        //Translation
        float diff_tra=(tra1-tra2).norm();

        tra_mean +=diff_tra;
    }

    tra_mean /= frames.size();

    return tra_mean;
}

float registrationErrorRot(vector< std::shared_ptr<PointCloud> >& frames){

    float rot_mean=0;

    for(std::shared_ptr<PointCloud> it : frames){
        Quaternionf rot1(it->pose.linear());
        Quaternionf rot2(it->poseGroundTruth.linear());

        //Rotation
        float d = rot1.dot(rot2);
        float diff_rot_degrees = rad2degM(acos(2*d*d - 1));

        rot_mean +=diff_rot_degrees;
    }

    rot_mean /= frames.size();

    return rot_mean;
}

}

double PointCloudManipulation::getPointCloudDiameter(PointCloud& C){

    //upper bound is the diagonal of enclosing cube
    Matrix3Xf m = C.ptsMat();
    Vector3f min = m.rowwise().minCoeff();
    Vector3f max = m.rowwise().maxCoeff();

    Vector3f corner = max - min;
    
    cout<<corner<<endl;
    
    double diagonal=corner.norm();
    cout<<"diagonalUpperBound:"<<diagonal<<endl;
    

    //for real diagonal, we have to loop n^2 times and compare each element with each other
    diagonal=0;
    
    for (int i=0; i<C.pts.size(); i++) {
        Vector3f& pt1=C.pts[i];
        for (int j=0; j<C.pts.size(); j++) {
            if(i==j) continue;
            Vector3f& pt2=C.pts[j];
            
            double distA=(pt2-pt1).norm();
            if (distA>diagonal) {
                diagonal=distA;
            }
        }
    }
    
    cout<<"diagonalExact:"<<diagonal<<endl;

    return diagonal;
}




//PointCloud PointCloudManipulation::projectPointsAndNormals(Isometry3f P, PointCloud CandN){
//    Matrix3Xf pts = vec2mat(CandN.pts);
//    Matrix3Xf nor = vec2mat(CandN.nor);

//    Matrix3Xf pts2 = P * pts;
//    Matrix3Xf nor2 = P.linear() * nor;

//    PointCloud C2;
//    CandN.pts=mat2vec(pts2);
//    CandN.nor=mat2vec(nor2);
    //C2.pts_color=CandN.pts_color;
    //C2.nor_color=CandN.pts_color;
    //C2.features=CandN.features;
    //C2.featuresComputed=CandN.featuresComputed;
//    C2.cur=CandN.cur;
//    C2.pts_color=CandN.pts_color;
//    C2.neighRadius=CandN.neighRadius;

    //cout<<"Projected "<<CandN.pts.size()<< "pts "<<endl;


//    return C2CandN;

//}

Matrix3f PointCloudManipulation::covarianceOfNeighbours(const vector<Vector3f>& pts, const Vector3f p1, const float neighRadius){
    //Matrix3f cov = Matrix3f::Zero();
    //Vector3f mean = Vector3f::Zero();

    vector<Vector3f> neighbours;

    int nN=0; //number of neighbours
    for (int j=0; j<pts.size(); j++) {
        Vector3f p2=pts[j];
        if ((p2-p1).norm()<neighRadius){
            neighbours.push_back(p2);
            nN++;
            //mean +=p2;
            //cov +=p1*p2.transpose(); //outer product
        }
    }

    //mean /= (nN+0.0f);
    //cov -= (mean * mean.transpose())*nN;


    MatrixXf mat(3,nN);
    int k=0;
    for(auto it : neighbours){
        mat.col(k++)=it;
    }

    //http://forum.kde.org/viewtopic.php?f=74&t=110265
    MatrixXf centered = mat.colwise() - mat.rowwise().mean();
    Matrix3f cov = centered * centered.adjoint();

    return cov;
}

vector<Vector3f> PointCloudManipulation::estimateNormals(const vector<Vector3f>& pts, const vector<Vector3f>& oldNormals, const float neighRadius){

    vector<Vector3f> nor;

    for (int i=0; i<pts.size(); i++) {
        const Vector3f& p1=pts[i];

        Matrix3f cov = covarianceOfNeighbours(pts,p1,neighRadius);

        //and then perform the eigendecomposition:
        SelfAdjointEigenSolver<MatrixXf> eig(cov);

        Vector3f normal=eig.eigenvectors().col(0);
        normal.normalize();

        //orient according to old normals
        if(oldNormals.size()>0){
            Vector3f oldNormal=oldNormals[i];
            double angle=acos(oldNormal.dot(normal));
            if(angle>=M_PI_2){
                //cout<<"otherside"<<endl;
                normal*=-1;
                //normal.normalize();
            }
        }else{ //TODO; orient according to viewpoint or away from given viewpont .... or even better: outwards on a surface

        }

        nor.push_back(normal);
    }

    return nor;
}

void PointCloudManipulation::reestimateNormals(PointCloud &C, const float neighRadius){
//    float min=std::numeric_limits<float>::max();
//    float max=std::numeric_limits<float>::min();

    if(C.nor.size()==0){
        cout<<"reestimateNormals, no previous normals"<<endl;
        C.nor=vector<Vector3f>(C.pts.size());
    }

    for (int i=0; i<C.pts.size(); i++) {
        Vector3f p1=C.pts[i];

        Matrix3f cov = covarianceOfNeighbours(C.pts,p1,neighRadius);

        //and then perform the eigendecomposition:
        SelfAdjointEigenSolver<MatrixXf> eig(cov);
        
        //From eig, you have access to the sorted (increasing order) eigenvalues (eig.eigenvalues()) and respective eigenvectors (eig.eigenvectors()). For instance, eig.eigenvectors().rightCols(N) gives you the best N-dimension basis.
        //cout<<"eigval/vec"<<endl;
        //cout<<eig.eigenvalues()<<endl;
        //cout<<eig.eigenvectors()<<endl;
        
        Vector3f normal=eig.eigenvectors().col(0);
        normal.normalize();

        //cout<<"cov"<<endl<<cov<<endl;
        //cout<<"eigvalues"<<endl<<eig.eigenvalues()<<endl;
        //cout<<"eigvectors"<<endl<<eig.eigenvectors()<<endl;

        //Vector3f eigVals = eig.eigenvalues();

        //curvature

    //    https://github.com/PointCloudLibrary/pcl/blob/647de7bed7df7cd383e5948ff42b116e5aae0e79/features/include/pcl/features/impl/feature.hpp#L82

//        float curvature = eigVals(0) / eigVals.sum();
//        if(curvature<min) min=curvature;
//        if(curvature>max) max=curvature;
//        C.cur.push_back(curvature);


        //orient according to old normals
        if(C.nor.size()>0){
            Vector3f oldNormal=C.nor[i];
            double angle=acos(oldNormal.dot(normal));
            if(angle>=M_PI_2){
                //cout<<"otherside"<<endl;
                normal*=-1;
                //normal.normalize();
            }
            C.nor[i]=normal;

        }else{ //TODO; orient according to viewpoint or away from given viewpont .... or even better: outwards on a surface
            Vector3f oldNormal(0,0,1); //positive z axis
            double angle=acos(oldNormal.dot(normal));
            cout<<"angle: "<<angle<<endl;
            if(angle>=M_PI_2){
                //cout<<"otherside"<<endl;
                normal*=-1;
                //normal.normalize();
            }
            C.nor.push_back(normal);
        }
        
        //cout<<normal<<endl;
    }

//    for(int i=0;i<C.cur.size();i++){
//        float gray=C.cur[i]/max;
//        C.pts_color.push_back(Colormap::Jet(gray));

//    }

  //  C.neighRadius=neighRadius;


   // cout<<"min:"<<min<<" max:"<<max<<endl;
    
    cout <<"Reestimated Normals and Curvature using PCA and Neighbours in a "<<neighRadius<<"m ball"<<endl;
}

//http://eigen.tuxfamily.org/dox/group__TutorialReductionsVisitorsBroadcasting.html   bottom example
int PointCloudManipulation::nearestNeighbourIdx(vector<Vector3f>& vec, Vector3f v){
    Matrix3Xf m = vec2mat(vec);
    MatrixXf::Index index;
      // find nearest neighbour
    (m.colwise() - v).colwise().squaredNorm().minCoeff(&index);

    return index;
}




Translation3f PointCloudManipulation::getTranslationToCentroid(PointCloud& C){

    Vector3f mean = PointCloudManipulation::getCentroid(C.pts);

    cout<<"Centroid at: "<< mean.x() <<" "<< mean.y() <<" "<< mean.z() <<endl;

    return Translation3f(-mean);
}

Vector3f PointCloudManipulation::getCentroid(vector<Vector3f>& pts){

    Matrix3Xf m = vec2mat(pts);
    Vector3f mean1=m.rowwise().mean();

//    Vector3f mean(0,0,0);
//    for (auto it : pts) {
//        mean+=it.cast<float>();
//    }
//    mean /= pts.size();
    return mean1;
}

Vector3f PointCloudManipulation::getNormal(vector<Vector3f>& pts){
    //http://forum.kde.org/viewtopic.php?f=74&t=110265
    Matrix3Xf mat = vec2mat(pts);

    MatrixXf centered = mat.colwise() - mat.rowwise().mean();
    Matrix3f cov = centered * centered.adjoint();

    SelfAdjointEigenSolver<Matrix3f> eig(cov);
    Vector3f no = eig.eigenvectors().col(0);
    if (no(2) > 0) no = -no;

    no.normalize();

    return no;
}





//every point in srcCloud is matched with the closest point to it in dstCloud, if this distance is smaller than thresh
//note: not every point in dstCloud is matched (e.g. back of bunny when srcCloud is a  frame)
//float PointCloudManipulation::getClosesPoints(PointCloud& srcCloud, vector<PointCloud>& dstClouds, vector<Vector3f> &src, vector<Vector3f> &dst,
//float thresh){
//    //vector<int> corresp;
//    float totalDistMeasure = 0;

//    for (int i = 0; i < srcCloud.pts.size(); ++i) {
//        Vector3f& srcPt = srcCloud.pts[i];
//        double diffMin = std::numeric_limits<double>::infinity();
//        int idxMin;
//        int kMin;

//        for(int k=0; k<dstClouds.size(); k++){
//            PointCloud& dstCloud=dstClouds[k];
//        for (int j = 0; j < dstCloud.pts.size(); ++j) {
//            Vector3f& dstPt = dstCloud.pts[j];
//            float diff = (srcPt-dstPt).norm();
//            if(diff<diffMin){
//                diffMin=diff;
//                idxMin=j;
//                kMin=k;
//            }
//        }
//        }

//        Vector3f& dstPtBest = dstClouds[kMin].pts[idxMin];

//        if(diffMin<thresh){ //get rid ouf outlier correspondences
//            //corresp.push_back(idxMin);
//            totalDistMeasure += (dstPtBest-srcPt).norm();
//            src.push_back(srcPt);
//            dst.push_back(dstPtBest);
//        }
//    }

//    totalDistMeasure /= src.size(); //mean value

//    return totalDistMeasure;
//}


//every point in srcCloud is matched with the closest point to it in dstCloud, if this distance is smaller than thresh
//note: not every point in dstCloud is matched (e.g. back of bunny when srcCloud is a  frame)
float PointCloudManipulation::getClosesPoints(PointCloud& srcCloud, PointCloud& dstCloud, vector<Vector3f> &src, vector<Vector3f> &dst,float thresh, bool useFlann, vector<Vector3f> &nor, vector<Vector3f> &norSrc){

    float totalDistMeasure = 0;

    Vector3f preTra = Vector3f(dstCloud.pose.translation());
    auto preInvRot = dstCloud.pose.linear().inverse();

   // timer.tic();

    for (int i = 0; i < srcCloud.pts.size(); ++i) {
        Vector3f& srcPtOrig = srcCloud.pts[i];
        Vector3f srcPtInGlobalFrame = srcCloud.pose*srcPtOrig;

        float diffMin;
        size_t idxMin;

        if(useFlann){
            Vector3f srcPtinDstFrame =  preInvRot * (srcPtInGlobalFrame-preTra);
            diffMin = sqrtf(dstCloud.getClosestPoint(srcPtinDstFrame,idxMin));
        }else{
            //linear search
            diffMin=std::numeric_limits<double>::infinity();
            for (int j = 0; j < dstCloud.pts.size(); ++j) {
                Vector3f& dstPt = dstCloud.pts[j];
                Vector3f dstPtInGlobalFrame = dstCloud.pose*dstPt;
                //Vector3f test = dstCloud.pose.linear().inverse() * (dstPtInGlobalFrame-Vector3f(dstCloud.pose.translation()));

                //float testdiff =  (dstPt-test2).norm();
                //cout<<"test diff" <<testdiff<<endl;

                float diff = (srcPtInGlobalFrame-dstPtInGlobalFrame).norm();
                if(diff<diffMin){
                    diffMin=diff;
                    idxMin=j;
                }
            }
        }

//        diffMin2=std::numeric_limits<double>::infinity();
//        for (int j = 0; j < dstCloud.pts.size(); ++j) {
//            Vector3f& dstPt = dstCloud.pts[j];

//            float diff = (srcPtinDstFrame-dstPt).norm();
//            if(diff<diffMin2){
//                diffMin2=diff;
//                idxMin2=j;
//            }
//        }


//        if(idxMin != idxMin2){
//            cout<<"BAD: idx1: "<<idxMin<<" idx2: "<<idxMin2<<" diffMin: "<<diffMin<<" diffMin2: "<<diffMin2<<endl;
//        }else{
//            cout<<"OK: idx1: "<<idxMin<<" idx2: "<<idxMin2<<" diffMin: "<<diffMin<<" diffMin2: "<<diffMin2<<endl;

//        }

        Vector3f dstPtInGlobalFrame = dstCloud.pose*dstCloud.pts[idxMin];

        if(diffMin<thresh){ //get rid ouf outlier correspondences
            //corresp.push_back(idxMin);
            totalDistMeasure += diffMin;//(dstPtBest-srcPt).norm();
            src.push_back(srcPtInGlobalFrame);
            dst.push_back(dstPtInGlobalFrame);
            if(dstCloud.nor.size()>0) nor.push_back(dstCloud.pose.linear()*dstCloud.nor[idxMin]);
            if(srcCloud.nor.size()>0) norSrc.push_back(srcCloud.pose.linear()*srcCloud.nor[idxMin]);

        }
    }

//    if(useFlann){
//        timer.toc("getClosesPoints flann");
//    }else{
//        timer.toc("getClosesPoints linear");
//    }

    return totalDistMeasure/src.size();
}

//Isometry3f ICP::computeStepUnordered(MatrixXf modelPoseEst, MatrixXf sSmall, float thresh){


//    //Isometry3f PPP=ICP::computeStep(src,dst,false);

//    return PPP;
//}
