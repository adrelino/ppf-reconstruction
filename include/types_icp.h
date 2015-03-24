#ifndef G2O_TYPES_ICP2
#define G2O_TYPES_ICP2

#define GICP_ANALYTIC_JACOBIANS2

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include <g2o/types/slam3d/vertex_se3.h>

#include <Eigen/Geometry>
#include <iostream>

namespace g2o2 {

using namespace g2o;

  namespace types_icp2 {
    void init();
  }

  typedef  Eigen::Matrix<double, 6, 1, Eigen::ColMajor> Vector6d;

//
// GICP-type edges
// Each measurement is between two rigid points on each 6DOF vertex
//

  //
  // class for edges between two points rigidly attached to vertices
  //

  class  EdgeGICP2
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

   public:
    // point positions
    Vector3D pos0, pos1;

    // unit normals
   // Vector3D normal0, normal1;

    // rotation matrix for normal
    //Matrix3D R0,R1;

    // initialize an object
    EdgeGICP2()
      {
        pos0.setZero();
        pos1.setZero();
        //normal0 << 0, 0, 1;
        //normal1 << 0, 0, 1;
        //makeRot();
        //R0.setIdentity();
        //R1.setIdentity();
      }

    /*
    // set up rotation matrix for pos0
    void makeRot0() 
    {
      Vector3D y;
      y << 0, 1, 0;
      R0.row(2) = normal0;
      y = y - normal0(1)*normal0;
      y.normalize();            // need to check if y is close to 0
      R0.row(1) = y;
      R0.row(0) = normal0.cross(R0.row(1));
      //      cout << normal.transpose() << endl;
      //      cout << R0 << endl << endl;
      //      cout << R0*R0.transpose() << endl << endl;
    }

    // set up rotation matrix for pos1
    void makeRot1()
    {
      Vector3D y;
      y << 0, 1, 0;
      R1.row(2) = normal1;
      y = y - normal1(1)*normal1;
      y.normalize();            // need to check if y is close to 0
      R1.row(1) = y;
      R1.row(0) = normal1.cross(R1.row(1));
    }

    // returns a precision matrix for point-plane
    Matrix3D prec0(double e)
    {
      makeRot0();
      Matrix3D prec;
      prec << e, 0, 0,
              0, e, 0,
              0, 0, 1;
      return R0.transpose()*prec*R0;
    }
    
    // returns a precision matrix for point-plane
    Matrix3D prec1(double e)
    {
      makeRot1();
      Matrix3D prec;
      prec << e, 0, 0,
              0, e, 0,
              0, 0, 1;
      return R1.transpose()*prec*R1;
    }
    
    // return a covariance matrix for plane-plane
    Matrix3D cov0(double e)
    {
      makeRot0();
      Matrix3D cov;
      cov  << 1, 0, 0,
              0, 1, 0,
              0, 0, e;
      return R0.transpose()*cov*R0;
    }
    
    // return a covariance matrix for plane-plane
    Matrix3D cov1(double e)
    {
      makeRot1();
      Matrix3D cov;
      cov  << 1, 0, 0,
              0, 1, 0,
              0, 0, e;
      return R1.transpose()*cov*R1;
    }

    */

  };

  using namespace std;

  // 3D rigid constraint
  //    3 values for position wrt frame
  //    3 values for normal wrt frame, not used here
  // first two args are the measurement type, second two the connection classes
  class  Edge_V_V_GICP2 : public  BaseBinaryEdge<3, EdgeGICP2, g2o::VertexSE3, VertexSE3>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Edge_V_V_GICP2() {}//: pl_pl(false) {}
    Edge_V_V_GICP2(const Edge_V_V_GICP2* e);

    // switch to go between point-plane and plane-plane
   // bool pl_pl;
//    Matrix3D cov0, cov1;

    // I/O functions
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 3-vector
    void computeError()
    {
      // from <ViewPoint> to <Point>
      const VertexSE3 *vp0 = static_cast<const VertexSE3*>(_vertices[0]);
      const VertexSE3 *vp1 = static_cast<const VertexSE3*>(_vertices[1]);

      // get vp1 point into vp0 frame
      // could be more efficient if we computed this transform just once
      Vector3D p1;

//#if 0
//      if (_cnum >= 0 && 0)      // using global cache
//        {
//          if (_tainted[_cnum])  // set up transform
//            {
//              _transforms[_cnum] = vp0->estimate().inverse() * vp1->estimate();
//              _tainted[_cnum] = 0;
//              cout << _transforms[_cnum] << endl;
//            }
//          p1 = _transforms[_cnum].map(measurement().pos1); // do the transform
//        }
//      else
//#endif
        {
          p1 = vp1->estimate() * measurement().pos1.cast<double>();
          p1 = vp0->estimate().inverse() * p1;
        }

//            cout << endl << "Error computation; points are: " << endl;
//            //cout << p0.transpose() << endl;
//            cout << p1.transpose() << endl;

      // get their difference
      // this is simple Euclidean distance, for now
      _error = p1 - measurement().pos0.cast<double>();

      //cout << "error: "<<_error.norm()<<endl;

//#if 0
//      cout << "vp0" << endl << vp0->estimate() << endl;
//      cout << "vp1" << endl << vp1->estimate() << endl;
//      cout << "e Jac Xj" << endl <<  _jacobianOplusXj << endl << endl;
//      cout << "e Jac Xi" << endl << _jacobianOplusXi << endl << endl;
//#endif

//      if (!pl_pl) return;

//      // re-define the information matrix
//      // topLeftCorner<3,3>() is the rotation()
//      const Matrix3D transform = ( vp0->estimate().inverse() *  vp1->estimate() ).matrix().topLeftCorner<3,3>();
//      information() = ( cov0 + transform * cov1 * transform.transpose() ).inverse();

    }

    // try analytic jacobians
#ifdef GICP_ANALYTIC_JACOBIANS2
    virtual void linearizeOplus();
#endif

    // global derivative matrices
    static Matrix3D dRidx;
	static Matrix3D dRidy;
	static Matrix3D dRidz; // differential quat matrices
  };



} // end namespace

#endif // TYPES_ICP
