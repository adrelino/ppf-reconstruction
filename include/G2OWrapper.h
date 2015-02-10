#ifndef G2OWRAPPER_H
#define G2OWRAPPER_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <Eigen/StdVector>
#include <random>
#include <iostream>
#include <stdint.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/icp/types_icp.h>


namespace g2oWrapper{
using namespace Eigen;
using namespace std;
using namespace g2o;

Isometry3f computeStep(vector<Vector3f> &src, vector<Vector3f> &srcNor, vector<Vector3f> &dst,vector<Vector3f> &dstNor){
    SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    // variable-size block solver
    BlockSolverX::LinearSolverType * linearSolver = new LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    BlockSolverX * solver_ptr = new BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);

    // set up two poses
    int vertex_id = 0;
    for (size_t i=0; i<2; ++i)
    {
      // set up rotation and translation for this node
      Vector3d t(0,0,0);
      Quaterniond q;
      q.setIdentity();

      Eigen::Isometry3d cam; // camera pose
      cam = q;
      cam.translation() = t;

      // set up node
      VertexSE3 *vc = new VertexSE3();
      vc->setEstimate(cam);

      vc->setId(vertex_id);      // vertex id

      //cerr << t.transpose() << " | " << q.coeffs().transpose() << endl;

      // set first cam pose fixed
      if (i==0)
        vc->setFixed(true);

      // add to optimizer
      optimizer.addVertex(vc);

      vertex_id++;
    }

    for (int i = 0; i < src.size(); ++i) {

        // get two poses
        VertexSE3* vp0 =
          dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second);
        VertexSE3* vp1 =
          dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second);

        Edge_V_V_GICP * e           // new edge with correct cohort for caching
            = new Edge_V_V_GICP();

        e->setVertex(0, vp0);      // first viewpoint

        e->setVertex(1, vp1);      // second viewpoint

        EdgeGICP meas;
        meas.pos0 = src[i].cast<double>();
        meas.pos1 = dst[i].cast<double>();
        meas.normal0 = srcNor[i].cast<double>();
        meas.normal1 = dstNor[i].cast<double>();

        e->setMeasurement(meas);
        //        e->inverseMeasurement().pos() = -kp;

        meas = e->measurement();
        // use this for point-plane
        e->information() = meas.prec0(0.01);

        // use this for point-point
        //    e->information().setIdentity();

        //    e->setRobustKernel(true);
        //e->setHuberWidth(0.01);

        optimizer.addEdge(e);
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    //cout << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

    //optimizer.setVerbose(true);

    optimizer.optimize(10);

//    cout << endl << "Second vertex should be near 0,0,1" << endl;
//    cout <<  dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second)
//      ->estimate().translation().transpose() << endl;
//    cout <<  dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second)
//      ->estimate().rotation() << endl;

//    cout<<"second"<<endl;
    auto second = dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second)->estimate();
//    cout <<  second.translation().transpose() << endl;
//    cout <<  second.rotation() << endl;



    return second.cast<float>().inverse();


}



}
#endif // G2OWRAPPER_H
