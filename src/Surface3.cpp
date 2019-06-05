#include "Surface3.h"

using namespace CalfFluidEngine;

Surface3::Surface3()
{
}


Surface3::~Surface3()
{
}

Vector3D CalfFluidEngine::Surface3::GetClosestPoint(const Vector3D & otherPoint) const
{
	return transform.TransformPoint(
		closestPointLocal(
			transform.InverseTransformPoint(otherPoint)
		)
	);
}

Vector3D CalfFluidEngine::Surface3::GetClosestNormal(const Vector3D & otherPoint) const
{
	Vector3D result = transform.TransformDirection(
		closestNormalLocal(
			transform.InverseTransformPoint(otherPoint)
		)
	);
	result *= (isNormalFlipped) ? -1.0 : 1.0;
	return result;
}

double CalfFluidEngine::Surface3::GetClosestDistance(const Vector3D & otherPoint) const
{
	return closestDistanceLocal(transform.InverseTransformPoint(otherPoint));
}

bool CalfFluidEngine::Surface3::Intersects(const Ray3D & ray) const
{
	SurfaceRayIntersection3 result = closestIntersectionLocal(
		transform.InverseTransformRay(ray)
	);
	return result.isIntersecting;
}

SurfaceRayIntersection3 CalfFluidEngine::Surface3::GetClosestIntersection(const Ray3D & ray)
{
	auto result = closestIntersectionLocal(transform.InverseTransformRay(ray));
	result.point = transform.TransformPoint(result.point);
	result.normal = transform.TransformDirection(result.normal);
	result.normal *= (isNormalFlipped) ? -1.0 : 1.0;
	return result;
}

bool CalfFluidEngine::Surface3::IsInside(const Vector3D & otherPoint) const
{
	return isNormalFlipped != 
		isInsideLocal(transform.InverseTransformPoint(otherPoint));
}

BoundingBox3D CalfFluidEngine::Surface3::GetBoundingBox() const
{
	return transform.TransformBoundingBox(getBoundingBoxLocal());
}

double CalfFluidEngine::Surface3::closestDistanceLocal(const Vector3D & otherPointLocal) const
{
	return Vector3D::Distance(
		otherPointLocal,
		closestPointLocal(otherPointLocal)
	);
}

bool CalfFluidEngine::Surface3::isInsideLocal(const Vector3D & otherPoint) const
{
	Vector3D cpLocal = closestPointLocal(otherPoint);
	Vector3D normalLocal = closestNormalLocal(otherPoint);
	return Vector3D::Dot((otherPoint - cpLocal),normalLocal) < 0.0;
}

