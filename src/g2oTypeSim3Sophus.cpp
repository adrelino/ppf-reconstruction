
#include "g2oTypeSim3Sophus.h"

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

namespace ppf_reconstruction
{


//G2O_USE_TYPE_GROUP(sba);

G2O_REGISTER_TYPE_GROUP(se3sophus);

G2O_REGISTER_TYPE(VERTEX_SE3_SOPHUS, VertexSe3);
G2O_REGISTER_TYPE(EDGE_ICP, EdgeICP);

VertexSe3::VertexSe3() : g2o::BaseVertex<6, Sophus::SE3d>()
{
	_marginalized=false;
}

bool VertexSe3::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;
}

bool VertexSe3::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
}


EdgeICP::EdgeICP() :
    g2o::BaseBinaryEdge<3, ClosesPointsCorrInGlobalFrame, VertexSe3, VertexSe3>()
{
}

bool EdgeICP::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;
}

bool EdgeICP::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
}

}
