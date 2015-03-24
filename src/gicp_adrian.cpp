// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <Eigen/StdVector>
#include <random>
#include <iostream>
#include <stdint.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/icp/types_icp.h>

#include "PointCloudManipulation.h"
#include "LoadingSaving.h"


//#include "types_icp.h"

using namespace Eigen;
using namespace std;
using namespace g2o;
//using namespace g2o2;

// sampling distributions
  class Sample
  {

    static default_random_engine gen_real;
    static default_random_engine gen_int;
  public:
    static int uniform(int from, int to);

    static double uniform();

    static double gaussian(double sigma);
  };


  default_random_engine Sample::gen_real;
  default_random_engine Sample::gen_int;

  int Sample::uniform(int from, int to)
  {
    uniform_int_distribution<int> unif(from, to);
    int sam = unif(gen_int);
    return  sam;
  }

  double Sample::uniform()
  {
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    double sam = unif(gen_real);
    return  sam;
  }

  double Sample::gaussian(double sigma)
  {
    std::normal_distribution<double> gauss(0.0, sigma);
    double sam = gauss(gen_real);
    return  sam;
  }


//
// set up simulated system with noise, optimize it
//

int main()
{
  double euc_noise = 0.01;       // noise in position, m
  //  double outlier_ratio = 0.1;


  SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  // variable-size block solver
  BlockSolverX::LinearSolverType * linearSolver = new LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
  BlockSolverX * solver_ptr = new BlockSolverX(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  optimizer.setAlgorithm(solver);

  vector<Vector3d> true_points;
  for (size_t i=0;i<1000; ++i)
  {
    true_points.push_back(Vector3d((Sample::uniform()-0.5)*3,
                                   Sample::uniform()-0.5,
                                   Sample::uniform()+10));
  }


  // set up two poses
  int vertex_id = 0;
  for (size_t i=0; i<2; ++i)
  {
    // set up rotation and translation for this node
    Vector3d t(0,0,i);
    Quaterniond q;
    q.setIdentity();

    Eigen::Isometry3d cam; // camera pose
    cam = q;
    cam.translation() = t;

    // set up node
    VertexSE3 *vc = new VertexSE3();
    vc->setEstimate(cam);

    vc->setId(vertex_id);      // vertex id

    cerr << t.transpose() << " | " << q.coeffs().transpose() << endl;

    // set first cam pose fixed
    if (i==0)
      vc->setFixed(true);

    // add to optimizer
    optimizer.addVertex(vc);

    vertex_id++;                
  }

  vector<Vector3f> src,dst;

  // set up point matches
  for (size_t i=0; i<true_points.size(); ++i)
  {
    // get two poses
    VertexSE3* vp0 = 
      dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second);
    VertexSE3* vp1 = 
      dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second);

    // calculate the relative 3D position of the point
    Vector3d pt0,pt1;
    pt0 = vp0->estimate().inverse() * true_points[i];
    pt1 = vp1->estimate().inverse() * true_points[i];

    // add in noise
    pt0 += Vector3d(Sample::gaussian(euc_noise ),
                    Sample::gaussian(euc_noise ),
                    Sample::gaussian(euc_noise ));

    pt1 += Vector3d(Sample::gaussian(euc_noise ),
                    Sample::gaussian(euc_noise ),
                    Sample::gaussian(euc_noise ));

    // form edge, with normals in varioius positions
    Vector3d nm0, nm1;
    nm0 << 0, i, 1;
    nm1 << 0, i, 1;
    nm0.normalize();
    nm1.normalize();

    Edge_V_V_GICP * e           // new edge with correct cohort for caching
        = new Edge_V_V_GICP();

    e->setVertex(0, vp0);      // first viewpoint

    e->setVertex(1, vp1);      // second viewpoint

    EdgeGICP meas;
    meas.pos0 = pt0;
    meas.pos1 = pt1;
    meas.normal0 = nm0;
    meas.normal1 = nm1;

    src.push_back(pt0.cast<float>());
    dst.push_back(pt1.cast<float>());

    e->setMeasurement(meas);
    //        e->inverseMeasurement().pos() = -kp;
    
    meas = e->measurement();
    // use this for point-plane
    //e->information() = meas.prec0(0.01);

    // use this for point-point 
       e->information().setIdentity();

    //    e->setRobustKernel(true);
    //e->setHuberWidth(0.01);

    optimizer.addEdge(e);
  }
  VertexSE3* vc0 = dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second);
  Eigen::Isometry3d cam0 = vc0->estimate();

  // move second cam off of its true position
  VertexSE3* vc = dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second);
  //vector<Isometry3f> posesGroundTruth = LoadingSaving::loadPoses(Params::getDir(),"pose");

  //Eigen::Isometry3d camOrig = vc->estimate();
  Eigen::Isometry3d cam;// = posesGroundTruth[8].cast<double>();//vc->estimate();
  cam = Translation3d(Vector3d(10,0.4,0.2))*AngleAxisd(1.5,Vector3d::UnitX())*AngleAxisd(0.6,Vector3d::UnitY())*AngleAxisd(50,Vector3d::UnitZ());
//  cam.translation() = ;
//  cam.rotation() = Quaterniond();
  vc->setEstimate(cam);

  //solver->setUserLambdaInit(400);
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  cerr << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

  optimizer.setVerbose(true);

  optimizer.optimize(200);


  //cout << endl << "Second vertex should be near 0,0,1" << endl;
  //cout <<  dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second)->estimate().translation().transpose() << endl;
  Isometry3d test = dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second)->estimate();
  //cout <<  test.translation().transpose() << endl;
  printPose(test,"test");

  float initialchi2=0;
  for (int i = 0; i < src.size(); ++i) {
      src[i] = cam0.cast<float>()*src[i];
      dst[i] = cam.cast<float>()*dst[i];
      Vector3f diff = src[i]-dst[i];
      initialchi2 += diff.dot(diff);
      //initialchi2 += diff.sum();
  }
  cout<<"initialchi2 icp: "<<initialchi2<<endl;

  Isometry3f step = ICP::pointToPoint(dst,src);  //src is fixed
  Isometry3d corr = step.cast<double>()*cam;
  printPose(corr,"corr");
  cout<<poseDiff(corr,test);

  float finalchi2=0;
  for (int i = 0; i < src.size(); ++i) {
      Vector3f diff = src[i]-step*dst[i];
      finalchi2 += diff.dot(diff);
      //initialchi2 += diff.sum();
  }
  cout<<"finalchi2 icp: "<<finalchi2<<endl;
}
