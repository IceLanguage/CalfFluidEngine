#include "Plane3.h"

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
