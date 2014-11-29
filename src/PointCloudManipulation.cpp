//
//  PointCloudManipulation.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "PointCloudManipulation.h"

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
    C2.cur=CandN.cur;
    C2.pts_color=CandN.pts_color;
    C2.neighRadius=CandN.neighRadius;

    cout<<"Projected "<<CandN.pts.size()<< "pts "<<endl;


    return C2;

//    int r=CandN.rows();
//    int c=CandN.cols();
//    MatrixXf C=CandN.block(0, 0, r, 3);
//    MatrixXf N=CandN.block(0, 3, r, 3);
    
//    VectorXf I=VectorXf::Ones(r);
//    VectorXf Z=VectorXf::Zero(r);
    
    
//    MatrixXf Ch(C.rows(), C.cols()+I.cols());
//    Ch << C, I;
//    //cout<<"N="<<N<<endl;
    
    
//    MatrixXf Nh(N.rows(), N.cols()+Z.cols());
//    Nh << N, Z;
    
//    MatrixXf m=P.matrix();
//    //cout<<"P="<<m<<endl;
    
//    MatrixXf C2=m*Ch.transpose();
    
//    //cout<<"C2="<<C2.transpose().row(0)<<endl;
    
//    MatrixXf N2=m*Nh.transpose();
//    //cout<<"N2="<<N2.transpose().row(0)<<endl;
    
//    MatrixXf C2i=C2.transpose().block(0, 0, r, 3);
//    //cout<<"C2i="<<C2i<<endl;
    
//    MatrixXf N2i=N2.transpose().block(0, 0, r, 3);
//    //cout<<"N2i="<<N2i<<endl;
    
    
    
    
//    MatrixXf CandN2(r, 6);
//    CandN2 << C2i, N2i;
    
    
//    //cout<<"C"<<C<<endl;
//    //cout<<"m"<<m<<"*"<<C.transpose();
    
//    //MatrixXf C2=P * C.transpose();
//    //MatrixXf N2=P.linear() * N.transpose();
    
//    //cout<<C2<<endl;
//    //cout<<N2<<endl;
    
    
//    return CandN2;
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
    float min=std::numeric_limits<float>::max();
    float max=std::numeric_limits<float>::min();

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

        Vector3f eigVals = eig.eigenvalues();

        //curvature

    //    https://github.com/PointCloudLibrary/pcl/blob/647de7bed7df7cd383e5948ff42b116e5aae0e79/features/include/pcl/features/impl/feature.hpp#L82

        float curvature = eigVals(0) / eigVals.sum();
        if(curvature<min) min=curvature;
        if(curvature>max) max=curvature;
        C.cur.push_back(curvature);


        //orient according to old normals
        if(C.nor.size()>0){
            Vector3f oldNormal=C.nor[i];
            double angle=acos(oldNormal.dot(normal));
            if(angle>=M_PI_2){
                //cout<<"otherside"<<endl;
                normal*=-1;
                //normal.normalize();
            }
        }else{ //TODO; orient according to viewpoint or away from given viewpont .... or even better: outwards on a surface

        }
        
        //cout<<normal<<endl;
        C.nor[i]=normal;
    }

    for(int i=0;i<C.cur.size();i++){
        float gray=C.cur[i]/max;
        C.pts_color.push_back(Colormap::Jet(gray));

    }

    C.neighRadius=neighRadius;


    cout<<"min:"<<min<<" max:"<<max<<endl;
    
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


PointCloud PointCloudManipulation::downSample(PointCloud C, bool useCenter){
    
    //MatrixXf scaled = C/ddist;
    unordered_map<string, vector<Vector3f> > voxels;
    
    for (int i=0; i<C.pts.size(); i++) {
        int x=round(C.pts[i].x()/ddist);
        int y=round(C.pts[i].y()/ddist);
        int z=round(C.pts[i].z()/ddist);
        stringstream ss;
        ss<<x<<"|"<<y<<"|"<<z;
        string key=ss.str();
        voxels[key].push_back(C.pts[i]);
    }
    
    PointCloud C2; // contains Cmean,Nmean,Ccenter,Ncenter
    
    int i=0;
    for (auto it : voxels){
        //        cout<<"key"<<it.first<<endl;
        //        cout<<"center"<<it.second.center<<endl;
        //        cout<<"pts:"<<endl;
        
        Vector3f voxelCenterOrPtsMean=PointCloudManipulation::getCentroid(it.second);

        C2.pts.push_back(voxelCenterOrPtsMean);


        //assign the normal from the point which was closest to this one before (since we use the mean or voxel center, this point didnt exist before, maybe using median would be better?)
        int idx = nearestNeighbourIdx(C.pts,voxelCenterOrPtsMean);
        C2.nor.push_back(C.nor[idx]);

        i++;
    }
    
    cout<<"DownSampled "<<C.pts.size()<<"->"<<C2.pts.size()<< " pts with voxelSize="<<ddist<<endl;
    
    return C2;
}

Translation3f PointCloudManipulation::getTranslationToCentroid(PointCloud C){

    Vector3f mean = PointCloudManipulation::getCentroid(C.pts);

    cout<<"Centroid at: "<< mean.x() <<" "<< mean.y() <<" "<< mean.z() <<endl;

    return Translation3f(-mean);
}

Vector3f PointCloudManipulation::getCentroid(vector<Vector3f> pts){
    Vector3f mean(0,0,0);
    for (auto it : pts) {
        mean+=it.cast<float>();
    }
    mean /= pts.size();
    return mean;
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

vector<int> PointCloudManipulation::getClosesPoints(PointCloud modelPoseEst, PointCloud sSmall, vector<Vector3f> &src, vector<Vector3f> &dst,
float thresh){
    vector<int> corresp;
    for (int i = 0; i < sSmall.rows(); ++i) {
        Vector3f scenePt = sSmall.pts[i];
        double diffMin = 9999999999999;//std::numeric_limits::infinity();
        int idxMin;
        for (int j = 0; j < modelPoseEst.rows(); ++j) {
            Vector3f modelPnt = modelPoseEst.pts[j];
            double diff = (scenePt-modelPnt).norm();
            if(diff<diffMin){
                diffMin=diff;
                idxMin=j;
            }
        }

        Vector3f modelPntBest = modelPoseEst.pts[idxMin];

        if(diffMin<thresh){ //get rid ouf outlier correspondences
            corresp.push_back(idxMin);
            src.push_back(modelPntBest);
            dst.push_back(scenePt);
        }
    }

    return corresp;
}

//Isometry3f ICP::computeStepUnordered(MatrixXf modelPoseEst, MatrixXf sSmall, float thresh){


//    //Isometry3f PPP=ICP::computeStep(src,dst,false);

//    return PPP;
//}


