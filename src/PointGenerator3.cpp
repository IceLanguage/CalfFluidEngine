#include "PointGenerator3.h"

using namespace CalfFluidEngine;

PointGenerator3::PointGenerator3()
{
}


PointGenerator3::~PointGenerator3()
{
}

void CalfFluidEngine::PointGenerator3::Generate(const BoundingBox3D & boundingBox, double spacing, std::vector<Vector3D>* points) const
{
	ForEachPoint(
		boundingBox,
		spacing,
		[&points](const Vector3D& point) {
			points->push_back(point);
			return true;
	});
}
