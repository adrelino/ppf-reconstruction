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

double PointCloudManipulation::getPointCloudDiameter(PointCloud C){

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
        Vector3f pt1=C.pts[i];
        for (int j=0; j<C.pts.size(); j++) {
            if(i==j) continue;
            Vector3f pt2=C.pts[j];
            
            double distA=(pt2-pt1).norm();
            if (distA>diagonal) {
                diagonal=distA;
            }
        }
    }
    
    cout<<"diagonalExact:"<<diagonal<<endl;

    return diagonal;
}




PointCloud PointCloudManipulation::projectPointsAndNormals(Isometry3f P, PointCloud CandN){
    Matrix3Xf pts = vec2mat(CandN.pts);
    Matrix3Xf nor = vec2mat(CandN.nor);

    Matrix3Xf pts2 = P * pts;
    Matrix3Xf nor2 = P.linear() * nor;

    PointCloud C2;
    C2.pts=mat2vec(pts2);
    C2.nor=mat2vec(nor2);
    C2.pts_color=CandN.pts_color;
//    C2.cur=CandN.cur;
//    C2.pts_color=CandN.pts_color;
//    C2.neighRadius=CandN.neighRadius;

    //cout<<"Projected "<<CandN.pts.size()<< "pts "<<endl;


    return C2;

}

Matrix3f PointCloudManipulation::covarianceOfNeighbours(const vector<Vector3f> pts, const Vector3f p1, const float neighRadius){
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

vector<Vector3f> PointCloudManipulation::estimateNormals(vector<Vector3f> pts, const vector<Vector3f> oldNormals, const float neighRadius){

    vector<Vector3f> nor;

    for (int i=0; i<pts.size(); i++) {
        Vector3f p1=pts[i];

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
int PointCloudManipulation::nearestNeighbourIdx(vector<Vector3f> vec, Vector3f v){
    Matrix3Xf m = vec2mat(vec);
    MatrixXf::Index index;
      // find nearest neighbour
    (m.colwise() - v).colwise().squaredNorm().minCoeff(&index);

    return index;
}


PointCloud PointCloudManipulation::downSample(PointCloud C, float voxelSize){
    
    //MatrixXf scaled = C/ddist;
    unordered_map<string, vector<Vector3f> > voxels;

    //bool withNormals=C.nor.size()>0;
    
    for (int i=0; i<C.pts.size(); i++) {
        int x=floor(C.pts[i].x()/voxelSize);
        int y=floor(C.pts[i].y()/voxelSize);
        int z=floor(C.pts[i].z()/voxelSize);
        stringstream ss;
        ss<<x<<"|"<<y<<"|"<<z;
        string key=ss.str();
        //int key = (x | (y<<10) | (z<<20));
//        if(voxels.count(key)==0){
//            voxels[key]=make_pair()
//        }
        voxels[key].push_back(C.pts[i]);
        //if(withNormals) voxels[key].second.push_back(C.nor[i]);
    }
    
    PointCloud C2; // contains Cmean,Nmean,Ccenter,Ncenter
    //C2.pts = vector<Vector3f>(voxels.size());
    //C2.nor = vector<Vector3f>(voxels.size());
    //int i=0;
    for (auto it : voxels){
        int npts = it.second.size();

                cout<<"key"<<it.first<<" #pts:"<<npts<<endl;

         if(npts<20) continue;
        
        Vector3f ptsMean=PointCloudManipulation::getCentroid(it.second);
        C2.pts.push_back(ptsMean);

        //does mean normal make sense?
        //if(withNormals){
        Vector3f norMean=PointCloudManipulation::getNormal(it.second);
        C2.nor.push_back(norMean);
        //}


        //assign the normal from the point which was closest to this one before (since we use the mean or voxel center, this point didnt exist before, maybe using median would be better?)
        //int idx = nearestNeighbourIdx(C.pts,voxelCenterOrPtsMean);
        //C2.nor.push_back(C.nor[idx]);

        //i++;
    }
    
    cout<<"DownSampled "<<C.pts.size()<<"->"<<C2.pts.size()<< " pts with voxelSize="<<voxelSize<<endl;
    
    return C2;
}

Translation3f PointCloudManipulation::getTranslationToCentroid(PointCloud C){

    Vector3f mean = PointCloudManipulation::getCentroid(C.pts);

    cout<<"Centroid at: "<< mean.x() <<" "<< mean.y() <<" "<< mean.z() <<endl;

    return Translation3f(-mean);
}

Vector3f PointCloudManipulation::getCentroid(vector<Vector3f> pts){

    Matrix3Xf m = vec2mat(pts);
    Vector3f mean1=m.rowwise().mean();

//    Vector3f mean(0,0,0);
//    for (auto it : pts) {
//        mean+=it.cast<float>();
//    }
//    mean /= pts.size();
    return mean1;
}

Vector3f PointCloudManipulation::getNormal(vector<Vector3f> pts){
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

//point to plane
Isometry3f ICP::computeStep(vector<Vector3f> &src,vector<Vector3f> &dst,vector<Vector3f> &nor)

{

    assert(src.size()==dst.size() && src.size()==nor.size());



// Maybe also have a look at that?

   // https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf



    // http://www.cs.princeton.edu/~smr/papers/icpstability.pdf

    Matrix<float,6,6> C;

    Matrix<float,6,1> b;

    C.setZero();

    b.setZero();

    for(uint i=0;i<src.size();++i)

    {

        Vector3f cro = src[i].cross(nor[i]);

        C.block<3,3>(0,0) += cro*cro.transpose();

        C.block<3,3>(0,3) += nor[i]*cro.transpose();

        C.block<3,3>(3,3) += nor[i]*nor[i].transpose();



        float sum = (src[i]-dst[i]).dot(nor[i]);

        b.head(3) -= cro*sum;

        b.tail(3) -= nor[i]*sum;

    }

    C.block<3,3>(3,0) = C.block<3,3>(0,3);

    Matrix<float,6,1> x = C.ldlt().solve(b);



    Isometry3f transform = Isometry3f::Identity();

    transform.linear() =

            (AngleAxisf(x(0), Vector3f::UnitX())

             * AngleAxisf(x(1), Vector3f::UnitY())

             * AngleAxisf(x(2), Vector3f::UnitZ())).toRotationMatrix();

    transform.translation() = x.block(3,0,3,1);



    return transform;

}


//point to point
//a,b sind centroids
Isometry3f ICP::computeStep(vector<Vector3f> &src,vector<Vector3f> &dst,Vector3f &a,Vector3f &b,bool withScale)
{
    assert(src.size()==dst.size());

    // http://www5.informatik.uni-erlangen.de/Forschung/Publikationen/2005/Zinsser05-PSR.pdf

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

Isometry3f ICP::computeStep(vector<Vector3f> &src, vector<Vector3f> &dst, bool withScale){
    Vector3f a = PointCloudManipulation::getCentroid(src);
    Vector3f b = PointCloudManipulation::getCentroid(dst);
    return computeStep(src,dst,a,b,withScale);
}

//every point in srcCloud is matched with the closest point to it in dstCloud, if this distance is smaller than thresh
//note: not every point in dstCloud is matched (e.g. back of bunny when srcCloud is a  frame)
vector<int> PointCloudManipulation::getClosesPoints(PointCloud srcCloud, PointCloud dstCloud, vector<Vector3f> &src, vector<Vector3f> &dst,
float thresh){
    vector<int> corresp;
    for (int i = 0; i < srcCloud.rows(); ++i) {
        Vector3f srcPt = srcCloud.pts[i];
        double diffMin = std::numeric_limits<double>::infinity();
        int idxMin;
        for (int j = 0; j < dstCloud.rows(); ++j) {
            Vector3f dstPt = dstCloud.pts[j];
            double diff = (srcPt-dstPt).norm();
            if(diff<diffMin){
                diffMin=diff;
                idxMin=j;
            }
        }

        Vector3f dstPtBest = dstCloud.pts[idxMin];

        if(diffMin<thresh){ //get rid ouf outlier correspondences
            corresp.push_back(idxMin);
            src.push_back(srcPt);
            dst.push_back(dstPtBest);
        }
    }

    return corresp;
}

//Isometry3f ICP::computeStepUnordered(MatrixXf modelPoseEst, MatrixXf sSmall, float thresh){


//    //Isometry3f PPP=ICP::computeStep(src,dst,false);

//    return PPP;
//}

Isometry3f ICP::getTransformationBetweenPointClouds(PointCloud model, PointCloud scene, int maxIter, float eps){

    Isometry3f P_Iterative_ICP = Isometry3f::Identity(); //initialize with ppf coarse alignment



        int j = 0;
        for (; j < maxIter; ++j) {
            vector<Vector3f> src,dst;

            PointCloud curr=PointCloudManipulation::projectPointsAndNormals(P_Iterative_ICP, model);
            PointCloudManipulation::getClosesPoints(curr,scene,src,dst,0.04f);

            //Visualize::setLines(src,dst);

            //cout<<"Iteration "<<j<<endl;
            cout<<"# Scene to Model Correspondences: "<<src.size()<<"=="<<dst.size()<<endl;

            //Visualize::spin(3);


            //cout<<"ICP "<<endl;

            Isometry3f P_incemental = ICP::computeStep(src,dst,false);
            //PointPairFeatures::printPose(P_incemental, "P incremental ICP");
            if(PointPairFeatures::isPoseCloseToIdentity(P_incemental,eps)){
                break;
            }
            P_Iterative_ICP = P_Iterative_ICP * P_incemental;


            //Visualize::setModelTransformed(curr);


        }

        cout<<"ICP converged after #steps="<<j<<endl;

        return P_Iterative_ICP;

}


