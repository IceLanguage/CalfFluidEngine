#include "Plane3.h"
#include <Constant.h>
using namespace CalfFluidEngine;

Plane3::Plane3()
{
}


Plane3::~Plane3()
{
}

bool CalfFluidEngine::Plane3::Intersects(const Ray3D & ray) const
{
	return std::fabs(Vector3D::Dot(ray.direction,_normal)) > 0;
}

Vector3D CalfFluidEngine::Plane3::closestPointLocal(const Vector3D & otherPoint) const
{
	float t = Vector3D::Dot(_normal, otherPoint) - _NormalDotPoint;
	return otherPoint - t * _normal;
}

Vector3D CalfFluidEngine::Plane3::closestNormalLocal(const Vector3D & otherPoint) const
{
	return _normal;
}

SurfaceRayIntersection3 CalfFluidEngine::Plane3::closestIntersectionLocal(const Ray3D & ray) const
{
	SurfaceRayIntersection3 intersection;

	double dDotN = Vector3D::Dot(ray.direction,_normal);
	double rayoriginDotN = Vector3D::Dot(ray.origin, _normal);
	// Check for vertical
	if (std::fabs(dDotN) > 0) {
		double t = (rayoriginDotN - _NormalDotPoint) / dDotN;
		if (t >= 0.0) {
			intersection.isIntersecting = true;
			intersection.distance = t;
			intersection.point = ray.GetPointAt(t);
			intersection.normal = _normal;
		}
	}

	return intersection;
}

BoundingBox3D CalfFluidEngine::Plane3::getBoundingBoxLocal() const
{
	static const double eps = kEpsilonD;
	static const double dmax = kMaxD;
	
	Vector3D point = Vector3D::zero;
	if (_normal.x != 0)
	{
		point.x = _NormalDotPoint / _normal.x;
	}
	else if(_normal.y != 0)
	{
		point.y = _NormalDotPoint / _normal.y;
	}
	else if (_normal.z != 0)
	{
		point.z = _NormalDotPoint / _normal.z;
	}

	if (std::fabs(Vector3D::Dot(_normal,Vector3D::right) - 1.0) < eps) {
		return BoundingBox3D(
			point - Vector3D(0, dmax, dmax),
			point + Vector3D(0, dmax, dmax));
	}
	else if (std::fabs(Vector3D::Dot(_normal, Vector3D::up) - 1.0) < eps) {
		return BoundingBox3D(
			point - Vector3D(dmax, 0, dmax),
			point + Vector3D(dmax, 0, dmax));
	}
	else if (std::fabs(Vector3D::Dot(_normal, Vector3D::forward) - 1.0) < eps) {
		return BoundingBox3D(
			point - Vector3D(dmax, dmax, 0),
			point + Vector3D(dmax, dmax, 0));
	}
	else {
		return BoundingBox3D(
			Vector3D(dmax, dmax, dmax),
			Vector3D(dmax, dmax, dmax));
	}
}
