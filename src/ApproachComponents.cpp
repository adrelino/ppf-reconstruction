#include "ApproachComponents.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
// #include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

#include <g2o/types/icp/types_icp.h>

#include <g2o/core/robust_kernel_impl.h>

//#include "types_icp.h"
//#include "g2oTypeSim3Sophus.h"
#include "Visualize.h"


using namespace Eigen;
using namespace std;
using namespace g2o;
//using namespace g2o2;

//using namespace ppf_reconstruction;
void ApproachComponents::g2oOptimizer(vector< std::shared_ptr<PointCloud> >& frames, float cutoff, int iter, int knn, double huberWidth, bool useLevenberg){

    g2o::RobustKernelHuber* rk;
    if(huberWidth>=0){
        rk = new g2o::RobustKernelHuber();
        rk->setDelta(huberWidth);
    }





//    for (int round = 0; round < iter; ++round) {

//    cout<<"g2oOptimizer round "<<round<<endl;
//    Visualize::spin();

    SparseOptimizer optimizer;
    //optimizer.setVerbose(true);
    // variable-size block solver
   // BlockSolverX::LinearSolverType * linearSolver = new LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
    BlockSolverX::LinearSolverType * linearSolver = new LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    BlockSolverX * solver_ptr = new BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithm* solver;
    if(useLevenberg){
    //g2o::OptimizationAlgorithmLevenberg*
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        //solver->setUserLambdaInit(1);

    }else{
        //g2o::OptimizationAlgorithmDogleg*
        solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
    }

        //g2o::OptimizationAlgorithm* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);




    optimizer.setAlgorithm(solver);

    //ParameterContainer ps = optimizer.parameters();
    //ps.write(std::cout);

    // set up all the poses
    //vector<VertexSE3*> vertices(frames.size());

    for(int i=0; i<frames.size(); i++){
      // set up node
      VertexSE3 *vc = new VertexSE3();
      //ppf_reconstruction::VertexSe3* vc = new ppf_reconstruction::VertexSe3();

      vc->setEstimate(frames[i]->pose.cast<double>());
      vc->setId(i);
      // set first cam pose fixed
      if (i==0){
          frames[i]->fixed=true;
          vc->setFixed(true);
      }

      // add to optimizer
      optimizer.addVertex(vc);
    }

    //cout<<"size 1: "<<vertices.size()<<" size2: "<<optimizer.vertices().size()<<endl;

    //float error=0;
    //float error2=0;
    for(int src_id=0; src_id<frames.size(); src_id++){
        PointCloud& srcCloud = *frames[src_id];
        //srcCloud.computePoseNeighbours(&frames,src_id,tra_thresh,180);
        //sif(round==0)
        srcCloud.computePoseNeighboursKnn(&frames,src_id,knn);

        srcCloud.computeClosestPointsToNeighbours(&frames,cutoff);
        //srcCloud.recalcError(&frames);
        //error2 +=srcCloud.accumError;

    }

    //cout<<"global error accum: "<<error<<" version 2 : "<<error2<<endl;

// if(stopStages){
//     cout<<"graph structure romputed, press q to refine"<<endl;
//     Visualize::spin();
// }



    for(int src_id=0; src_id<frames.size(); src_id++){

        PointCloud& srcCloud = *frames[src_id];

        for (int j = 0; j < srcCloud.neighbours.size(); ++j) {

            OutgoingEdge& dstEdge = srcCloud.neighbours[j];
            PointCloud& dstCloud = *frames.at(dstEdge.neighbourIdx);

            int dst_id=dstEdge.neighbourIdx;

            VertexSE3* vp0 =
              dynamic_cast<VertexSE3*>(optimizer.vertices().find(dst_id)->second); //dstCloud
            VertexSE3* vp1 =
              dynamic_cast<VertexSE3*>(optimizer.vertices().find(src_id)->second); //srcCloud


            for(auto corr : dstEdge.correspondances){
                Edge_V_V_GICP * e = new Edge_V_V_GICP();
                //ppf_reconstruction::EdgeICP* e = new ppf_reconstruction::EdgeICP();

                e->setVertex(0, vp0);      // first viewpoint : dstcloud, fixed
                e->setVertex(1, vp1);      // second viewpoint: srcCloud, moves

//                ppf_reconstruction::ClosesPointsCorrInGlobalFrame meas;
//                meas.dstPt=dstCloud.pts[corr.second].cast<double>();
//                meas.srcPt=srcCloud.pts[corr.first].cast<double>();

                EdgeGICP meas;
                meas.pos0 = dstCloud.pts[corr.second].cast<double>();
                meas.pos1 = srcCloud.pts[corr.first].cast<double>();
                meas.normal0 = dstCloud.nor[corr.second].cast<double>();
                meas.normal1 = srcCloud.nor[corr.first].cast<double>();

                meas.makeRot0();
                meas.makeRot1();

                if(huberWidth>=0){
                    e->setRobustKernel(rk);
                }



                e->setMeasurement(meas);


                //    e->setRobustKernel(true);
                //e->setHuberWidth(0.01);


                bool planeToPlane=false;



                if(planeToPlane){
                    e->pl_pl=true;
                }else if(Params::getInstance()->pointToPlane){
                    //meas = e->measurement();
                    // use this for point-plane
                    e->information() = meas.prec0(0.01);
                }else{
                    // use this for point-point
                    e->information().setIdentity();
                }

                optimizer.addEdge(e);

            }
        }
    }




//    optimizer.computeActiveErrors();
//    cerr << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

//    optimizer.setVerbose(true);

     //solver->setUserLambdaInit(0.66);
//    optimizer.optimize(1);


    //solver->setUserLambdaInit(0.001);

   optimizer.initializeOptimization();

   optimizer.computeActiveErrors();
   double chiInit = optimizer.chi2(); //stop innerround if we get 100% better
   cout << "round: " << "s" << " chi2: " << FIXED(chiInit) << endl;

   double lastChi=chiInit;

   int noImpr=0;


    for (int innerround = 0; innerround < iter; ++innerround) {


        //solver->setUserLambdaInit(0.001);

       //optimizer.initializeOptimization();



       optimizer.optimize(100);

       optimizer.computeActiveErrors();
       double newchi=optimizer.chi2();
       double impr = (lastChi-newchi)/lastChi;  //http://en.wikipedia.org/wiki/Relative_change_and_difference
       //lastChi/newchi

       lastChi=newchi;

       cout << "round: " << innerround << " chi2: " << FIXED(newchi) << " impr: "<<impr<<endl;

       if(impr>0.0){
           cout<<"impr > 0%"<<endl;
           if(Params::getInstance()->stopFrames){
               for (int i = 0; i < frames.size(); ++i) {
                   frames[i]->pose= dynamic_cast<VertexSE3*>(optimizer.vertices().find(i)->second)->estimate().cast<float>();
               }
               cout<<"press q to continue"<<endl;

               //Visualize::simulateKeypress('k'); //calc error
               Visualize::spinToggle(1);
            }
       }else{
           noImpr++;
       }

       if(noImpr>10){
           cout<<"10 times no impr, break";
           break;
       }



        //float chi2=0;


        //cerr<<"errrorrrr: "<<chi2<<endl;

//        if(newchi / chiInit < 0.01){
//            cout<<"exit innerround because of 100% improvement"<<endl;

//            break;
//        }


//        cout<<"optimized iter: "<<innerround<<endl;
//        Visualize::spinToggle(2);

    }

    for (int i = 0; i < frames.size(); ++i) {
        frames[i]->pose= dynamic_cast<VertexSE3*>(optimizer.vertices().find(i)->second)->estimate().cast<float>();
        //frames[i]->recalcError(&frames);
        //chi2+=frames[i]->accumError;
    }

}

#include "Visualize.h"
#include "LoadingSaving.h"
#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
#include <iostream>
#include <string>

void ApproachComponents::preprocessing(std::vector<std::shared_ptr<PointCloud> > &frames, int step, bool showDepthMap, int limit){
    vector<string> images = LoadingSaving::getAllImagesFromFolder(Params::getDir(),"*depth");
    vector<string> imagesMasks = LoadingSaving::getAllImagesFromFolder(Params::getDir(),"*mask");
    vector<string> intrinsics = LoadingSaving::getAllTextFilesFromFolder(Params::getDir(),"Intrinsic");

    if(intrinsics.size()!=1){
        cerr<<"can't find Intrinisc.txt file"<<endl;
        exit(1);
    }
    Matrix3f K = LoadingSaving::loadMatrix3f(intrinsics[0]);

    vector<Isometry3f> posesGroundTruth = LoadingSaving::loadPoses(Params::getDir(),"pose");

    int i_frame=0;
    int start = 0;

    for(int i=start; i<images.size() && i_frame<limit; i+=step, i_frame++){

        string maskname = ""; //no mask
        if(i<imagesMasks.size()){
            maskname=imagesMasks[i];
        }

        shared_ptr<PointCloud> currentFrame(new PointCloud(images[i],K,maskname,showDepthMap));

        Isometry3f P = Isometry3f::Identity();
        //Transformation groundTruth
        if(i<posesGroundTruth.size()){
            P = posesGroundTruth[i];
            currentFrame->setPoseGroundTruth(P);
        }

        //Transformation estimate
        Isometry3f P_est = Isometry3f::Identity();
        //get inter frame motion
        if(i==start) P_est=P;

        currentFrame->setPose(P_est);
        currentFrame->imgSequenceIdx=i;

        currentFrame->downsample(Params::getInstance()->ddist);
        frames.push_back(currentFrame);
    }
}

void ApproachComponents::alignFrame(std::vector<std::shared_ptr<PointCloud> > &frames, int nFrames, int i_frame){
    std::shared_ptr<PointCloud> currentFrame = frames[i_frame];
    Isometry3f pose = currentFrame->pose;

   // cout<<"[Frame "<<i_frame<<" Image "<<frames[i_frame]->imgSequenceIdx<<"] ";

    if(Params::getInstance()->stopFrames){
        Visualize::setSelectedIndex(i_frame);
    }

    Isometry3f P_relative;

    //coarse
    int ppfMaxVotes = -1;
    int jBest = -1;

    for(int j=i_frame-1; j>=i_frame-nFrames && j>=0;j--){ //consider last nFrames at most
        Poses poses = PointPairFeatures::getTransformationBetweenPointClouds(*currentFrame,*frames[j]); //frames.back() for drift
        if(poses[0].second>=ppfMaxVotes){ //ppfMinVotes){
            ppfMaxVotes=poses[0].second;
            P_relative=poses[0].first;
            jBest=j;
        }
    }

    currentFrame->neighboursDescr="votes";//ss.str();
    currentFrame->neighbours.clear();

    OutgoingEdge edge;
    edge.neighbourIdx=jBest;
    edge.weight=ppfMaxVotes;
    edge.P_relative = P_relative;
    currentFrame->neighbours.push_back(edge);

    frames[jBest]->children.push_back(i_frame);

    if(Params::getInstance()->stopFrames){
        Visualize::getInstance()->ingoingEdgeFrame=jBest;
    }


    //P_relative is motion from jBest -> i, so actually it is scene to model
    pose = frames[jBest]->pose * P_relative; //absolute pose for visualizatoin
    currentFrame->setPose(pose);

    cout<<"[Frame "<<i_frame<<"] [PPF to Frame "<<jBest<<"] \tmaxVotes: "<<ppfMaxVotes<<endl;


}

void ApproachComponents::pairwiseAlignment(std::vector<std::shared_ptr<PointCloud> > &frames, int nFrames){

    for(int i_frame=1; i_frame<frames.size(); i_frame++){
        alignFrame(frames,nFrames,i_frame);
        if(Params::getInstance()->stopFrames){
            Visualize::spinToggle(1);
        }
    }
}

void ApproachComponents::pairwiseAlignmentAndRefinement(std::vector<std::shared_ptr<PointCloud> > &frames, int nFrames, float cutoff ){

    for(int i_frame=1; i_frame<frames.size(); i_frame++){
        alignFrame(frames,nFrames,i_frame);
        std::shared_ptr<PointCloud> currentFrame = frames[i_frame];
        //if(Params::getInstance()->stopFrames) Visualize::setSelectedIndex(i_frame);
            int j;
            float icpInlierError;
            for (j=0; j < 8;j++) {  //ICP
                icpInlierError = currentFrame->computeClosestPointsToNeighbours(&frames,cutoff);
                if(currentFrame->alignToFirstNeighbourWithICP(&frames,Params::getInstance()->pointToPlane,false)) break;
                if(Params::getInstance()->stopFrames) Visualize::spin(1);
            }
            cout<<"[Frame "<<i_frame<<"] [ICP itertions "<<j<<"]"<<"\t ICP dist error:"<<icpInlierError<<endl;

            if(Params::getInstance()->stopFrames) Visualize::spin(1);
    }
}

void ApproachComponents::pairwiseRefinement(std::vector<std::shared_ptr<PointCloud> > &frames, float cutoff){

    for(int i_frame=1; i_frame<frames.size(); i_frame++){
        std::shared_ptr<PointCloud> currentFrame = frames[i_frame];
        if(Params::getInstance()->stopFrames) Visualize::setSelectedIndex(i_frame);

            int j;
            float icpInlierError;
            for (j=0; j < 8;j++) {  //ICP
                icpInlierError = currentFrame->computeClosestPointsToNeighboursRelative(&frames,cutoff);
                //if(Params::getInstance()->stopFrames) Visualize::spinToggle(1);
                if(currentFrame->alignToFirstNeighbourWithICP(&frames,Params::getInstance()->pointToPlane,true)) break;
            }
            cout<<"[Frame "<<i_frame<<"] [ICP itertions "<<j<<"]"<<"\t ICP dist error:"<<icpInlierError<<endl;
        if(Params::getInstance()->stopFrames){
                    currentFrame->updateChildrenAbsolutePoses(frames,i_frame);  // update absolute poses of children of this node in dependecy tree
                    Visualize::spinToggle(5);
        }

    }

}
