#include "Visualize.h"
#include "LoadingSaving.h"
#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
#include <iostream>
#include <string>
#include "Constants.h"

using namespace std;

#include "se3.hpp"

void deriveErrorNumeric(const vector<Vector3f>& ptsRef, const vector<Vector3f>& ptsProj, MatrixXf& Jac, VectorXf& res){
    float eps = 1e-8;

    for(int i=0; i<ptsRef.size(); i++){
        Vector3f diffVec = ptsProj[i] - ptsRef[i];
        res(i)=diffVec.dot(diffVec);

        for(int j=0; j<6; j++){
            Sophus::Vector6f epsVec=Sophus::Vector6f::Zero();
            epsVec(j) = eps;

            Vector3f ptsProjNearby = Sophus::SE3f::exp(epsVec)*ptsProj[i];
            Vector3f diff = ptsProjNearby - ptsRef[i];
            float resDiff = diff.dot(diff) - res(i);
            Jac(i,j) = resDiff / eps;
        }

    }

}

void deriveErrorAnalytic(const vector<Vector3f>& ptsRef, const vector<Vector3f>& ptsProj, MatrixXf& Jac, VectorXf& res){

    for(int i=0; i<ptsRef.size(); i++){
        MatrixXf d(3,6);

        Matrix3f hatMatI;

        Vector3f diffVec = ptsProj[i]-ptsRef[i];

        const Vector3f& x=ptsProj[i];// v1->pts[corr.first]; //posev1Inv * ptsProj;
        hatMatI<< 0.f,    x(2), -x(1),
              -x(2),  0.f ,   x(0),
              x(1), -x(0), 0.f;

        d << Matrix3f::Identity(), hatMatI;

        Jac.row(i) = diffVec.transpose() * d;
        res(i)=diffVec.dot(diffVec);
       //cout<<Jac.row(j)<<"\t | \t"<<res(j)<<endl;
    }

}


int main(int argc, char * argv[])
{

    int stop = 2;
    getParam("stop",stop,argc,argv);

    int knn=6;
    getParam("knn",knn,argc,argv);

    float cutoff=0.03;
    getParam("cutoff", cutoff, argc, argv);



    Params* inst = Params::getInstance();
    inst->getParams(argc,argv);

    vector<string> images = LoadingSaving::getAllImagesFromFolder(Params::getInstance()->dir,"depth");
    vector<string> imagesMasks = LoadingSaving::getAllImagesFromFolder(Params::getDir(),"*mask");

    vector<Isometry3f> posesGroundTruth = LoadingSaving::loadPoses(Params::getInstance()->dir,"pose");
    vector<Isometry3f> posesEst = LoadingSaving::loadPosesFromDir(Params::getInstance()->dir,Params::getInstance()->est_poses_prefix); //estimated poses
    vector<string> posesEstFileNames = LoadingSaving::getAllTextFilesFromFolder(Params::getInstance()->dir,Params::getInstance()->est_poses_prefix);

    if(posesEstFileNames.size()==0){
        cerr<<"you need to run frameToFrame first to get a first pose estimate"<<endl;
        exit(1);
    }

    vector<string> intrinsics = LoadingSaving::getAllTextFilesFromFolder(Params::getInstance()->dir,"Intrinsic");
    Matrix3f K = LoadingSaving::loadMatrix3f(intrinsics[0]);

    vector< std::shared_ptr<PointCloud> > frames;

    Visualize::setClouds(&frames);

    for(int i_frame=0; i_frame<posesEstFileNames.size() && frames.size()<stop; i_frame++){

        if(i_frame==1) i_frame--;

        string estimate = posesEstFileNames[i_frame];
        string number = estimate.substr(estimate.length()-Params::getInstance()->est_poses_suffix.length()-6,6);

        int i = atoi(number.c_str());
        cout<<"est "<<estimate<<" number "<<i<<endl;

        string maskname = ""; //no mask
        if(i<imagesMasks.size()){
            maskname=imagesMasks[i];
        }

        shared_ptr<PointCloud> currentFrame(new PointCloud(images[i],K,maskname));

        //currentFrame->downsample(Params::getInstance()->ddist);

        //Transformation groundTruth
        Isometry3f P = posesGroundTruth[i];
        currentFrame->setPoseGroundTruth(P);

        //Transformation estimate
        Isometry3f P_est = posesEst[i_frame];

        currentFrame->setPose(P_est);


        frames.push_back(currentFrame);
    }

   // if(i_frame==1){
    Isometry3f P_est = posesEst[0];
        auto bla = P_est.translation();
        bla.z() += 0.04;
        P_est.translation()=bla;
        //Quaternionf quat(P_est.rotation());
        Matrix3f m(AngleAxisf(0.03,Vector3f::UnitX()) * AngleAxisf(0.6,Vector3f::UnitY()));
        Matrix3f blu = P_est.rotation();
        P_est.matrix().block<3,3>(0,0)=blu*m;
     frames[1]->pose=P_est;
   // }

    //Visualize::spin();


    float error=0;
    float error2=0;

    for(int src_id=1; src_id<frames.size(); src_id++){
        PointCloud& srcCloud = *frames[src_id];

        srcCloud.neighbours.push_back({0,4});

        for(int i=0; i<srcCloud.pts.size(); i++){
            srcCloud.neighbours[0].correspondances.push_back(make_pair(i,i));
        }


//        srcCloud.computePoseNeighboursKnn(&frames,src_id,knn);
//        error += srcCloud.computeClosestPointsToNeighbours(&frames,cutoff);
//        srcCloud.recalcError(&frames);
//        error2 +=srcCloud.accumError;

    }


    for(int round = 0 ; round < 20; round++){






    cout<<"global error accum: "<<error<<" version 2 : "<<error2<<endl;
    Visualize::spin();


    shared_ptr<PointCloud>& v1=frames[1];

    OutgoingEdge pair = v1->neighbours[0];
    int j=pair.neighbourIdx;

    shared_ptr<PointCloud>& v2=frames[j];

//    vector<Vector3f> src,dst;
//    for(auto corr : pair.correspondances){
//        src.push_back(v1->pts[corr.first]);
//        dst.push_back(v2->pts[corr.second]);
//    }

    //Isometry3f P = v1->pose;

    //align src (frame1) to dst (frame0)

    /*
     *         auto srcMat = vec2mat(src);
        auto srcMatProj = Test * srcMat;
        vector<Vector3f> pts = mat2vec(srcMatProj);
        */

    //Sophus::Vector6f xi = Sophus::Vector6f::Zero();
    //Sophus::Vector6f xi = Sophus::SE3f::log(Sophus::SE3f(P.matrix()));

    for(int i=0; i<50; i++){
        //v1->pose = Eigen::Isometry3f( Sophus::SE3f::exp(xi).matrix() );

        //cout<<"poseDiff_toGroundTruth: "<<poseDiff(v1->poseGroundTruth,v1->pose)<<endl;

        MatrixXf Jac = MatrixXf(pair.correspondances.size(),6);
        VectorXf res = VectorXf(pair.correspondances.size());
        vector<Vector3f> ptsProj(pair.correspondances.size());
        vector<Vector3f> ptsRef(pair.correspondances.size());

        for(int j=0; j<pair.correspondances.size(); j++){
            auto corr = pair.correspondances[j];
            ptsProj[j] = v1->pose * v1->pts[corr.first];
            ptsRef[j] = v2->pose * v2->pts[corr.second];
        }

        deriveErrorNumeric(ptsRef, ptsProj,Jac,res);
        //deriveErrorAnalytic(ptsRef,ptsProj,Jac,res);

        float err = res.dot(res);


//        cout<<Jac<<endl;
//        cout<<res<<endl;

        Sophus::Vector6f upd;

        float lambda = 0.01f;

        Matrix<float,6,6> R=Matrix<float,6,6>::Identity();
        R *=lambda;


//        if(i<25){
            upd = - ((Jac.transpose() * Jac + R).inverse() * Jac.transpose() * res);
//        }else{
            //Gauss-newton step
            //gradient descent step
//            upd =  - (0.001) * Jac.transpose() * res;
//        }



        //cout<<"upd: "<<upd<<endl;
        //left multiply increment
        //xi = Sophus::SE3f::log(Sophus::SE3f::exp(upd) * Sophus::SE3f::exp(xi));

        Eigen::Isometry3f updIso(Sophus::SE3f::exp(upd).matrix() );

        v1->pose = updIso * v1->pose;

        cout<<"error: "<<err<<endl;
        Visualize::spinToggle(2);




    }

    cout<<"round "<<round<<" finished"<<endl;
    Visualize::spin();

    }


}

