#include "SurfaceSet3.h"

using namespace CalfFluidEngine;

SurfaceSet3::SurfaceSet3()
{
}


SurfaceSet3::~SurfaceSet3()
{
}

void CalfFluidEngine::SurfaceSet3::AddSurface(const std::shared_ptr<Surface3>& surface)
{
	_surfaces.push_back(surface);
}

Vector3D CalfFluidEngine::SurfaceSet3::closestPointLocal(const Vector3D & otherPoint) const
{
	buildBVH();

	const auto distanceFunc = [](
		const std::shared_ptr<Surface3>& surface,
		const Vector3D& pt) {
		return surface->GetClosestDistance(pt);
	};

	Vector3D result{ kMaxD, kMaxD, kMaxD };
	const auto queryResult = _bvh.GetNearest(otherPoint, distanceFunc);
	if (queryResult.item != nullptr) {
		result = (*queryResult.item)->GetClosestPoint(otherPoint);
	}

	return result;
}

void CalfFluidEngine::SurfaceSet3::buildBVH() const
{
	std::vector<BoundingBox3D> bounds;
	for (size_t i = 0; i < _surfaces.size(); ++i) {
		bounds.push_back(_surfaces[i]->GetBoundingBox());
	}
	_bvh.Build(_surfaces, bounds);
}
