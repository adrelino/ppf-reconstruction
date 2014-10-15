//
//  PointCloudManipulation.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "PointCloudManipulation.h"

double PointCloudManipulation::getPointCloudDiameter(MatrixXd m){
    VectorXd X=m.col(0);
    VectorXd Y=m.col(1);
    VectorXd Z=m.col(2);
    
    //cout<<X<<endl;
    
    double x=X.maxCoeff()-X.minCoeff();
    double y=Y.maxCoeff()-Y.minCoeff();
    double z=Z.maxCoeff()-Z.minCoeff();
    
    cout<<x<<"*"<<y<<"*"<<z<<endl;
    
    Vector3d corner(x,y,z);
    
    cout<<corner<<endl;
    
    double diagonal=corner.norm();
    cout<<"diagonal:"<<diagonal<<endl;
    cout<<"ddist:"<<0.05*diagonal<<endl;
    
    diagonal=0;
    
    for (int i=0; i<m.rows(); i++) {
        RowVector3d pt1=m.block(i,0,1,3);
        for (int j=0; j<m.rows(); j++) {
            if(i==j) continue;
            RowVector3d pt2=m.block(j,0,1,3);
            
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




MatrixXd PointCloudManipulation::projectPointsAndNormals(Transform<double,3,Projective> P, MatrixXd CandN){
    int r=CandN.rows();
    int c=CandN.cols();
    MatrixXd C=CandN.block(0, 0, r, 3);
    MatrixXd N=CandN.block(0, 3, r, 3);
    
    VectorXd I=VectorXd::Ones(r);
    VectorXd Z=VectorXd::Zero(r);
    
    
    MatrixXd Ch(C.rows(), C.cols()+I.cols());
    Ch << C, I;
    //cout<<"N="<<N<<endl;
    
    
    MatrixXd Nh(N.rows(), N.cols()+Z.cols());
    Nh << N, Z;
    
    MatrixXd m=P.matrix();
    //cout<<"P="<<m<<endl;
    
    MatrixXd C2=P*Ch.transpose();
    
    //cout<<"C2="<<C2.transpose().row(0)<<endl;
    
    MatrixXd N2=P*Nh.transpose();
    //cout<<"N2="<<N2.transpose().row(0)<<endl;
    
    MatrixXd C2i=C2.transpose().block(0, 0, r, 3);
    //cout<<"C2i="<<C2i<<endl;
    
    MatrixXd N2i=N2.transpose().block(0, 0, r, 3);
    //cout<<"N2i="<<N2i<<endl;
    
    
    
    
    MatrixXd CandN2(r, 6);
    CandN2 << C2i, N2i;
    
    
    //cout<<"C"<<C<<endl;
    //cout<<"m"<<m<<"*"<<C.transpose();
    
    //MatrixXd C2=P * C.transpose();
    //MatrixXd N2=P.linear() * N.transpose();
    
    //cout<<C2<<endl;
    //cout<<N2<<endl;
    
    cout<<"Projected "<<r<< "pts with transform:"<<endl<<m<<endl;
    
    return CandN2;
}

MatrixXd PointCloudManipulation::reestimateNormals(MatrixXd C){
    
    double neighbourBallSize=ddist*2; //neighbors in 2*ddist ball around p1 (e.g. 2cm)
    
    for (int i=0; i<C.rows(); i++) {
        RowVector3d p1=C.block(i, 0, 1, 3);
        vector<RowVector3d> neighbors;
        //auto indices = norm(C)
        for (int j=0; j<C.rows(); j++) {
            RowVector3d p2=C.block(j, 0, 1, 3);
            if ((p2-p1).norm()<neighbourBallSize){
                neighbors.push_back(p2);
            }
        }
        
        if(neighbors.size()<3){
            C.block(i, 3, 1, 3)=RowVector3d(0,0,0);
            cout<<"no pca: i="<<i<<endl;
            continue;
        }
        
        MatrixXd mat(neighbors.size(),3);
        
        int k=0;
        for(auto it : neighbors){
            mat.row(k++)=it;
        }
        
        //http://forum.kde.org/viewtopic.php?f=74&t=110265
        
        MatrixXd centered = mat.rowwise() - mat.colwise().mean();
        MatrixXd cov = centered.adjoint() * centered;
        
        //and then perform the eigendecomposition:
        
        SelfAdjointEigenSolver<MatrixXd> eig(cov);
        
        //From eig, you have access to the sorted (increasing order) eigenvalues (eig.eigenvalues()) and respective eigenvectors (eig.eigenvectors()). For instance, eig.eigenvectors().rightCols(N) gives you the best N-dimension basis.
        //cout<<"eigval/vec"<<endl;
        //cout<<eig.eigenvalues()<<endl;
        //cout<<eig.eigenvectors()<<endl;
        
        RowVector3d oldNormal=C.block(i, 3, 1, 3);
        oldNormal.normalize();
        RowVector3d normal=eig.eigenvectors().col(0).transpose();
        normal.normalize();
        
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
    
    cout <<"Reestimated Normals using PCA and Neighbours in a "<<neighbourBallSize<<"m ball"<<endl;
    
    return C;
}



struct Voxel {RowVector3d center; vector<RowVectorXd> pts;};
typedef std::pair<string,Voxel> myPair;


MatrixXd PointCloudManipulation::downSample(MatrixXd C, bool useCenter){
    
    //MatrixXd scaled = C/ddist;
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
            RowVector3d voxelCenter;
            voxelCenter(0)=x*ddist;
            voxelCenter(1)=y*ddist;
            voxelCenter(2)=z*ddist;
            voxels[key].center=voxelCenter;
        }
    }
    
    MatrixXd C2(voxels.size(),6); // contains Cmean,Nmean,Ccenter,Ncenter
    
    int i=0;
    for (auto it : voxels){
        //        cout<<"key"<<it.first<<endl;
        //        cout<<"center"<<it.second.center<<endl;
        //        cout<<"pts:"<<endl;
        
        RowVector3d voxelCenterOrPtsMean;
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
        
        double dist=9999999;
        RowVector3d normal;
        for (auto it2: it.second.pts){
            RowVector3d pt2=it2.head(3);
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
    
    return reestimateNormals(C2);
}

MatrixXd PointCloudManipulation::translateCentroidToOrigin(MatrixXd C){
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

    MatrixXd C2(C.rows(),6); // contains Cmean,Nmean,Ccenter,Ncenter

    for (int i=0; i<C2.rows(); i++) {
        C2(i,0)=C(i,0)-x;
        C2(i,1)=C(i,1)-y;
        C2(i,2)=C(i,2)-z;
        C2.block(i,3,1,3)=C.block(i,3,1,3); //normal is unchanged by pure translation
    }

    return C2;
}

