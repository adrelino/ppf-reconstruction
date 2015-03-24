
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
//#include "g2o/types/sba/types_six_dof_expmap.h"

#include "se3.hpp"
#include "PointCloud.h"


namespace ppf_reconstruction
{


class VertexSe3 : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexSe3();

	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl() {
        _estimate = Sophus::SE3d();
	}

    virtual void oplusImpl(const double* update_)
	{
        Eigen::Map< Eigen::Matrix<double, 6, 1> > update(const_cast<double*>(update_));

        setEstimate(Sophus::SE3d::exp(update) * estimate());
	}

    void setEstimateIso(Isometry3f pose){
        Isometry3d poseD = pose.cast<double>();
        _estimate = Sophus::SE3d(poseD.linear(),poseD.translation());
    }

    Isometry3f getEstimateIso(){
        return Isometry3f(_estimate.matrix().cast<float>());
    }
};

class ClosesPointsCorrInGlobalFrame{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Vector3d srcPt; //align srcPt to dstPt
    Vector3d dstPt; //fixed
};

/**
* \brief 3D edge between two Vertex6
*/
class EdgeICP : public g2o::BaseBinaryEdge<3, ClosesPointsCorrInGlobalFrame, VertexSe3, VertexSe3>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeICP();
	
	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;
	
	void computeError()
	{
        const VertexSe3* dst = static_cast<const VertexSe3*>(_vertices[0]); //dst fixed
        const VertexSe3* src = static_cast<const VertexSe3*>(_vertices[1]); //src moves

        Vector3d ptsProj = src->estimate() * _measurement.srcPt;
        Vector3d ptsRef =  dst->estimate() * _measurement.dstPt;

        _error=  ptsProj - ptsRef;

	}
	
//	void linearizeOplus()
//	{
//        const VertexSe3* _from = static_cast<const VertexSe3*>(_vertices[0]);

//        _jacobianOplusXj << Matrix3d::Identity,hat(_measurement.ptsRef).cast<double>();
//        _jacobianOplusXi << Matrix3d::Identity,hat(_measurement.pts).cast<double>();
//	}


    virtual void setMeasurement(const ClosesPointsCorrInGlobalFrame& m)
	{
		_measurement = m;
	}

};

}
