#include "Sphere3.h"

using namespace CalfFluidEngine;

Sphere3::Sphere3()
{
}


Sphere3::~Sphere3()
{
}

Vector3D CalfFluidEngine::Sphere3::closestPointLocal(const Vector3D & otherPoint) const
{
	return radius * closestNormalLocal(otherPoint) + center;
}

Vector3D CalfFluidEngine::Sphere3::closestNormalLocal(const Vector3D & otherPoint) const
{
	if (center.IsSimilar(otherPoint)) {
		return Vector3D(1, 0, 0);
	}
	else {
		Vector3D v = otherPoint - center;
		v.Normalize();
		return v;
	}
}

SurfaceRayIntersection3 CalfFluidEngine::Sphere3::closestIntersectionLocal(const Ray3D & ray) const
{
	SurfaceRayIntersection3 intersection;
	Vector3D r = ray.origin - center;
	double b = Vector3D::Dot(ray.direction,r);
	double c = r.SquareMagnitude() - b * b;
	double d = radius * radius - c;
	d = std::sqrt(d);

	if (d > 0.) {
		d = std::sqrt(d);
		double tMin = -b - d;
		double tMax = -b + d;

		if (tMin < 0.0) {
			tMin = tMax;
		}

		if (tMin < 0.0) {
			intersection.isIntersecting = false;
		}
		else {
			intersection.isIntersecting = true;
			intersection.distance = tMin;
			intersection.point = ray.origin + tMin * ray.direction;
			intersection.normal = Vector3D::Normalize(intersection.point - center);
		}
	}
	else {
		intersection.isIntersecting = false;
	}

	return intersection;
}

double CalfFluidEngine::Sphere3::closestDistanceLocal(const Vector3D & otherPointLocal) const
{
	return std::fabs(Vector3D::Distance(center, otherPointLocal) - radius);
}

BoundingBox3D CalfFluidEngine::Sphere3::getBoundingBoxLocal() const
{
	Vector3D r(radius, radius, radius);
	return BoundingBox3D(center - r, center + r);
}
