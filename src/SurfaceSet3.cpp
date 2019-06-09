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

bool CalfFluidEngine::SurfaceSet3::Intersects(const Ray3D & ray) const
{
	buildBVH();

	const auto testFunc = [](const std::shared_ptr<Surface3>& surface, const Ray3D& ray) {
		return surface->Intersects(ray);
	};

	bool result = _bvh.Intersects(ray, testFunc);
	return result;
}

void CalfFluidEngine::SurfaceSet3::Update()
{
	buildBVH();
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

Vector3D CalfFluidEngine::SurfaceSet3::closestNormalLocal(const Vector3D & otherPoint) const
{
	buildBVH();

	const auto distanceFunc = [](
		const std::shared_ptr<Surface3>& surface,
		const Vector3D& pt) {
		return surface->GetClosestDistance(pt);
	};

	Vector3D result{ 1.0, 0.0, 0.0 };
	const auto queryResult = _bvh.GetNearest(otherPoint, distanceFunc);
	if (queryResult.item != nullptr) {
		result = (*queryResult.item)->GetClosestNormal(otherPoint);
	}

	return result;
}

SurfaceRayIntersection3 CalfFluidEngine::SurfaceSet3::closestIntersectionLocal(const Ray3D & ray) const
{
	buildBVH();

	const auto testFunc = [](const std::shared_ptr<Surface3>& surface, const Ray3D& ray) {
		SurfaceRayIntersection3 result = surface->GetClosestIntersection(ray);
		return result.distance;
	};

	const auto queryResult = _bvh.GetClosestIntersection(ray, testFunc);
	SurfaceRayIntersection3 result;
	result.distance = queryResult.distance;
	result.isIntersecting = queryResult.item != nullptr;
	if (queryResult.item != nullptr) {
		result.point = ray.GetPointAt(queryResult.distance);
		result.normal = (*queryResult.item)->GetClosestNormal(result.point);
	}

	return result;
}

BoundingBox3D CalfFluidEngine::SurfaceSet3::getBoundingBoxLocal() const
{
	buildBVH();

	return _bvh.GetBoundingBox();
}

double CalfFluidEngine::SurfaceSet3::closestDistanceLocal(const Vector3D & otherPointLocal) const
{
	buildBVH();

	const auto distanceFunc = [](const std::shared_ptr<Surface3>& surface,
		const Vector3D& pt) {
		return surface->GetClosestDistance(pt);
	};

	const auto queryResult = _bvh.GetNearest(otherPointLocal, distanceFunc);

	double minDist = queryResult.distance;

	return minDist;
}

bool CalfFluidEngine::SurfaceSet3::isInsideLocal(const Vector3D & otherPoint) const
{
	for (auto surface : _surfaces) {
		if (surface->IsInside(otherPoint)) {
			return true;
		}
	}

	return false;
}

double CalfFluidEngine::SurfaceSet3::signedDistanceLocal(const Vector3D & otherPoint) const
{
	double sdf = kMaxD;
	for (const auto& surface : _surfaces) {
		sdf = std::min(sdf, surface->SignedDistance(otherPoint));
	}

	return sdf;
}

void CalfFluidEngine::SurfaceSet3::buildBVH() const
{
	std::vector<BoundingBox3D> bounds;
	for (size_t i = 0; i < _surfaces.size(); ++i) {
		bounds.push_back(_surfaces[i]->GetBoundingBox());
	}
	_bvh.Build(_surfaces, bounds);
}
