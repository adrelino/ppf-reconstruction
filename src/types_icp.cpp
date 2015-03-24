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

#include "types_icp.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

using namespace Eigen;

namespace g2o2 {

  G2O_REGISTER_TYPE_GROUP(icp);
  G2O_REGISTER_TYPE(EDGE_V_V_GICP2, Edge_V_V_GICP2);

  namespace types_icp2 {
    int initialized = 0;

    void init()
    {
      if (types_icp2::initialized)
        return;
      //cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;

      Edge_V_V_GICP2::dRidx << 0.0,0.0,0.0,
        0.0,0.0,2.0,
        0.0,-2.0,0.0;
      Edge_V_V_GICP2::dRidy  << 0.0,0.0,-2.0,
        0.0,0.0,0.0,
        2.0,0.0,0.0;
      Edge_V_V_GICP2::dRidz  << 0.0,2.0,0.0,
        -2.0,0.0,0.0,
        0.0,0.0,0.0;

      types_icp2::initialized = 1;
    }
  }

  using namespace std;
  using namespace Eigen;
  typedef Matrix<double, 6, 1, Eigen::ColMajor> Vector6d;

  Matrix3D Edge_V_V_GICP2::dRidx; // differential quat matrices
  Matrix3D Edge_V_V_GICP2::dRidy; // differential quat matrices
  Matrix3D Edge_V_V_GICP2::dRidz; // differential quat matrices

  // global initialization
  G2O_ATTRIBUTE_CONSTRUCTOR(init_icp_types)
  {
    types_icp2::init();
  }

  // Copy constructor
  Edge_V_V_GICP2::Edge_V_V_GICP2(const Edge_V_V_GICP2* e)
    : BaseBinaryEdge<3, EdgeGICP2, VertexSE3, VertexSE3>()
  {

    // Temporary hack - TODO, sort out const-ness properly
    _vertices[0] = const_cast<HyperGraph::Vertex*> (e->vertex(0));
    _vertices[1] = const_cast<HyperGraph::Vertex*> (e->vertex(1));

    _measurement.pos0 = e->measurement().pos0;
    _measurement.pos1 = e->measurement().pos1;
//    _measurement.normal0 = e->measurement().normal0;
//    _measurement.normal1 = e->measurement().normal1;
//    _measurement.R0 = e->measurement().R0;
//    _measurement.R1 = e->measurement().R1;

//    pl_pl = e->pl_pl;
//    cov0 = e->cov0;
//    cov1 = e->cov1;

    // TODO the robust kernel is not correctly copied
    //_robustKernel = e->_robustKernel;
  }

  //
  // Rigid 3D constraint between poses, given fixed point offsets
  //

  // input two matched points between the frames
  // first point belongs to the first frame, position and normal
  // second point belongs to the second frame, position and normal
  //
  // the measurement variable has type EdgeGICP (see types_icp.h)

  bool Edge_V_V_GICP2::read(std::istream& is)
  {
    return true;
  }


  // Jacobian
  // [ -R0'*R1 | R0 * dRdx/ddx * 0p1 ]
  // [  R0'*R1 | R0 * dR'dx/ddx * 0p1 ]

#ifdef GICP_ANALYTIC_JACOBIANS2

  // jacobian defined as:
  //    f(T0,T1) =  dR0.inv() * T0.inv() * (T1 * dR1 * p1 + dt1) - dt0
  //    df/dx0 = [-I, d[dR0.inv()]/dq0 * T01 * p1]
  //    df/dx1 = [R0, T01 * d[dR1]/dq1 * p1]
  void Edge_V_V_GICP2::linearizeOplus()
  {
    cout<<"analytic jacobian"<<endl;
    VertexSE3* vp0 = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3* vp1 = static_cast<VertexSE3*>(_vertices[1]);

    // topLeftCorner<3,3>() is the rotation matrix
    Matrix3D R0T = vp0->estimate().matrix().topLeftCorner<3,3>().transpose();
    Vector3D p1 = measurement().pos1;

    // this could be more efficient
    if (!vp0->fixed())
      {
        Isometry3D T01 = vp0->estimate().inverse() *  vp1->estimate();
        Vector3D p1t = T01 * p1;
        _jacobianOplusXi.block<3,3>(0,0) = -Matrix3D::Identity();
        _jacobianOplusXi.block<3,1>(0,3) = dRidx*p1t;
        _jacobianOplusXi.block<3,1>(0,4) = dRidy*p1t;
        _jacobianOplusXi.block<3,1>(0,5) = dRidz*p1t;

        std::cout<<"_opluxxi:"<<std::endl;
        std::cout<<_jacobianOplusXi<<std::endl;
      }

    if (!vp1->fixed())
      {
        Matrix3D R1 = vp1->estimate().matrix().topLeftCorner<3,3>();
        R0T = R0T*R1;
        _jacobianOplusXj.block<3,3>(0,0) = R0T;
        _jacobianOplusXj.block<3,1>(0,3) = R0T*dRidx.transpose()*p1;
        _jacobianOplusXj.block<3,1>(0,4) = R0T*dRidy.transpose()*p1;
        _jacobianOplusXj.block<3,1>(0,5) = R0T*dRidz.transpose()*p1;
        std::cout<<"_opluxxj:"<<std::endl;
        std::cout<<_jacobianOplusXj<<std::endl;

      }
  }
#endif


  bool Edge_V_V_GICP2::write(std::ostream& os) const
  {
    return os.good();
  }


} // end namespace
