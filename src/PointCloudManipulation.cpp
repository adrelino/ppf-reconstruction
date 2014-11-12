//
//  PointCloudManipulation.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "PointCloudManipulation.h"

double PointCloudManipulation::getPointCloudDiameter(MatrixXf m){
    VectorXf X=m.col(0);
    VectorXf Y=m.col(1);
    VectorXf Z=m.col(2);
    
    //cout<<X<<endl;
    
    double x=X.maxCoeff()-X.minCoeff();
    double y=Y.maxCoeff()-Y.minCoeff();
    double z=Z.maxCoeff()-Z.minCoeff();
    
    cout<<x<<"*"<<y<<"*"<<z<<endl;
    
    Vector3f corner(x,y,z);
    
    cout<<corner<<endl;
    
    double diagonal=corner.norm();
    cout<<"diagonal:"<<diagonal<<endl;
    cout<<"ddist:"<<0.05*diagonal<<endl;
    
    diagonal=0;
    
    for (int i=0; i<m.rows(); i++) {
        RowVector3f pt1=m.block(i,0,1,3);
        for (int j=0; j<m.rows(); j++) {
            if(i==j) continue;
            RowVector3f pt2=m.block(j,0,1,3);
            
            double distA=(pt2-pt1).norm();
            if (distA>diagonal) {
                diagonal=distA;
            }
        }
    }
    
    cout<<"diagonal:"<<diagonal<<endl;
    cout<<"ddist:"<<0.05*diagonal<<endl;


    
    return 0;
}




MatrixXf PointCloudManipulation::projectPointsAndNormals(Isometry3f P, MatrixXf CandN){
    int r=CandN.rows();
    int c=CandN.cols();
    MatrixXf C=CandN.block(0, 0, r, 3);
    MatrixXf N=CandN.block(0, 3, r, 3);
    
    VectorXf I=VectorXf::Ones(r);
    VectorXf Z=VectorXf::Zero(r);
    
    
    MatrixXf Ch(C.rows(), C.cols()+I.cols());
    Ch << C, I;
    //cout<<"N="<<N<<endl;
    
    
    MatrixXf Nh(N.rows(), N.cols()+Z.cols());
    Nh << N, Z;
    
    MatrixXf m=P.matrix();
    //cout<<"P="<<m<<endl;
    
    MatrixXf C2=m*Ch.transpose();
    
    //cout<<"C2="<<C2.transpose().row(0)<<endl;
    
    MatrixXf N2=m*Nh.transpose();
    //cout<<"N2="<<N2.transpose().row(0)<<endl;
    
    MatrixXf C2i=C2.transpose().block(0, 0, r, 3);
    //cout<<"C2i="<<C2i<<endl;
    
    MatrixXf N2i=N2.transpose().block(0, 0, r, 3);
    //cout<<"N2i="<<N2i<<endl;
    
    
    
    
    MatrixXf CandN2(r, 6);
    CandN2 << C2i, N2i;
    
    
    //cout<<"C"<<C<<endl;
    //cout<<"m"<<m<<"*"<<C.transpose();
    
    //MatrixXf C2=P * C.transpose();
    //MatrixXf N2=P.linear() * N.transpose();
    
    //cout<<C2<<endl;
    //cout<<N2<<endl;
    
    //cout<<"Projected "<<r<< "pts with transform:"<<endl<<m<<endl;
    
    return CandN2;
}

pair<MatrixXf,VectorXf> PointCloudManipulation::reestimateNormals(MatrixXf C){
    VectorXf curvatures(C.rows());
        
    for (int i=0; i<C.rows(); i++) {
        RowVector3f p1=C.block(i, 0, 1, 3);
        vector<RowVector3f> neighbors;
        //auto indices = norm(C)
        for (int j=0; j<C.rows(); j++) {
            RowVector3f p2=C.block(j, 0, 1, 3);
            if ((p2-p1).norm()<neighbourBallSize){
                neighbors.push_back(p2);
            }
        }
        
        if(neighbors.size()<3){
            C.block(i, 3, 1, 3)=RowVector3f(0,0,0);
            cout<<"no pca: i="<<i<<endl;
            continue;
        }
        
        MatrixXf mat(neighbors.size(),3);
        
        int k=0;
        for(auto it : neighbors){
            mat.row(k++)=it;
        }
        
        //http://forum.kde.org/viewtopic.php?f=74&t=110265
        
        MatrixXf centered = mat.rowwise() - mat.colwise().mean();
        MatrixXf cov = centered.adjoint() * centered;
        
        //and then perform the eigendecomposition:
        
        SelfAdjointEigenSolver<MatrixXf> eig(cov);
        
        //From eig, you have access to the sorted (increasing order) eigenvalues (eig.eigenvalues()) and respective eigenvectors (eig.eigenvectors()). For instance, eig.eigenvectors().rightCols(N) gives you the best N-dimension basis.
        //cout<<"eigval/vec"<<endl;
        //cout<<eig.eigenvalues()<<endl;
        //cout<<eig.eigenvectors()<<endl;
        
        RowVector3f oldNormal=C.block(i, 3, 1, 3);
        oldNormal.normalize();
        RowVector3f normal=eig.eigenvectors().col(0).transpose();
        normal.normalize();

        //cout<<"cov"<<endl<<cov<<endl;
        //cout<<"eigvalues"<<endl<<eig.eigenvalues()<<endl;
        //cout<<"eigvectors"<<endl<<eig.eigenvectors()<<endl;

        Vector3f eigVals = eig.eigenvalues();

        //curvature

    //    https://github.com/PointCloudLibrary/pcl/blob/647de7bed7df7cd383e5948ff42b116e5aae0e79/features/include/pcl/features/impl/feature.hpp#L82

        float curvature = eigVals(0) / eigVals.sum();
        //cout<<curvature<<endl;
        curvatures(i)=curvature;

        
        //orient according to old normals
        double angle=acos(oldNormal.dot(normal));
        if(angle>M_PI_2){
            //cout<<"otherside"<<endl;
            normal*=-1;
            normal.normalize();
        }
        
        //cout<<normal<<endl;
        C.block(i, 3, 1, 3)=normal;
    }
    
    cout <<"Reestimated Normals and Curvature using PCA and Neighbours in a "<<neighbourBallSize<<"m ball"<<endl;
    
    return {C,curvatures};
}



struct Voxel {RowVector3f center; vector<RowVectorXf> pts;};
typedef std::pair<string,Voxel> myPair;


MatrixXf PointCloudManipulation::downSample(MatrixXf C, bool useCenter){
    
    //MatrixXf scaled = C/ddist;
    unordered_map<string, Voxel> voxels;
    
    for (int i=0; i<C.rows(); i++) {
        int x=round(C(i,0)/ddist);
        int y=round(C(i,1)/ddist);
        int z=round(C(i,2)/ddist);
        stringstream ss;
        ss<<x<<"|"<<y<<"|"<<z;
        string key=ss.str();
        voxels[key].pts.push_back(C.row(i));
        
        if(useCenter){
            //TODO calculated multiple times(reduntand)
            RowVector3f voxelCenter;
            voxelCenter(0)=x*ddist;
            voxelCenter(1)=y*ddist;
            voxelCenter(2)=z*ddist;
            voxels[key].center=voxelCenter;
        }
    }
    
    MatrixXf C2(voxels.size(),6); // contains Cmean,Nmean,Ccenter,Ncenter
    
    int i=0;
    for (auto it : voxels){
        //        cout<<"key"<<it.first<<endl;
        //        cout<<"center"<<it.second.center<<endl;
        //        cout<<"pts:"<<endl;
        
        RowVector3f voxelCenterOrPtsMean;
        if(useCenter){
            voxelCenterOrPtsMean=it.second.center;
        }else{ //mean
            double x,y,z;
            x=y=z=0;
            for (auto it2 : it.second.pts){
                //cout<<it2<<endl;
                x+=it2(0);y+=it2(1);z+=it2(2);
            }
            long s=it.second.pts.size();
            x/=s;y/=s;z/=s;
            //        cout<<"s:"<<s<<endl;
            voxelCenterOrPtsMean<<x,y,z;
            //        cout<<"mean:"<<mean<<endl;
        }
        
        
        
        //voxelCenter << C.row(i).head(3); //normal pt
        
        double dist=std::numeric_limits<double>::max();
        RowVector3f normal;
        for (auto it2: it.second.pts){
            RowVector3f pt2=it2.head(3);
            double nDist=(voxelCenterOrPtsMean-pt2).norm();
            if(nDist<dist){
                dist=nDist;
                normal=it2.tail(3);  //assign the normal from the point which was closest to this one before (since we use the mean or voxel center, this point didnt exist before, maybe using median would be better?)
            }
        }
        
        
        C2.block(i,0,1,3)=voxelCenterOrPtsMean;
        C2.block(i,3,1,3)=normal;
        
        
        
        i++;
    }
    
    cout<<"DownSampled "<<C.rows()<<"->"<<C2.rows()<< " pts with voxelSize="<<ddist<<"m using "<<(useCenter ? "Voxelcenter" : "Mean of pts per Voxel")<<endl;
    
    return reestimateNormals(C2).first;
}

Translation3f PointCloudManipulation::getTranslationToCentroid(MatrixXf C){
    double x=0;
    double y=0;
    double z=0;

    for (int i=0; i<C.rows(); i++) {
        x+=C(i,0);
        y+=C(i,1);
        z+=C(i,2);
    }
    x/=C.rows();
    y/=C.rows();
    z/=C.rows();

    cout<<"Centroid at: "<< x <<" "<< y <<" "<< z <<endl;

    return Translation3f(-x,-y,-z);
}

Vector3f PointCloudManipulation::getCentroid(vector<Vector3f> pts){
    Vector3f mean(0,0,0);
    for (auto it : pts) {
        mean+=it.cast<float>();
    }
    mean /= pts.size();
    return mean;
}


MatrixXf PointCloudManipulation::translateCentroidToOrigin(MatrixXf C){
    double x=0;
    double y=0;
    double z=0;

    for (int i=0; i<C.rows(); i++) {
        x+=C(i,0);
        y+=C(i,1);
        z+=C(i,2);
    }
    x/=C.rows();
    y/=C.rows();
    z/=C.rows();

    cout<<"Centroid at: "<< x <<" "<< y <<" "<< z <<endl;

    MatrixXf C2(C.rows(),6); // contains Cmean,Nmean,Ccenter,Ncenter

    for (int i=0; i<C2.rows(); i++) {
        C2(i,0)=C(i,0)-x;
        C2(i,1)=C(i,1)-y;
        C2(i,2)=C(i,2)-z;
        C2.block(i,3,1,3)=C.block(i,3,1,3); //normal is unchanged by pure translation
    }

    return C2;
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

vector<int> PointCloudManipulation::getClosesPoints(MatrixXf modelPoseEst, MatrixXf sSmall, vector<Vector3f> &src, vector<Vector3f> &dst,
float thresh){
    vector<int> corresp;
    for (int i = 0; i < sSmall.rows(); ++i) {
        RowVector3f scenePt = sSmall.block(i,0,1,3);
        double diffMin = 9999999999999;//std::numeric_limits::infinity();
        int idxMin;
        for (int j = 0; j < modelPoseEst.rows(); ++j) {
            RowVector3f modelPnt = modelPoseEst.block(j,0,1,3);
            double diff = (scenePt-modelPnt).norm();
            if(diff<diffMin){
                diffMin=diff;
                idxMin=j;
            }
        }

        RowVector3f modelPntBest = modelPoseEst.block(idxMin,0,1,3);

        if(diffMin<thresh){ //get rid ouf outlier correspondences
            corresp.push_back(idxMin);
            src.push_back(modelPntBest.adjoint());
            dst.push_back(scenePt.adjoint());
        }
    }

    return corresp;
}

//Isometry3f ICP::computeStepUnordered(MatrixXf modelPoseEst, MatrixXf sSmall, float thresh){


//    //Isometry3f PPP=ICP::computeStep(src,dst,false);

//    return PPP;
//}


