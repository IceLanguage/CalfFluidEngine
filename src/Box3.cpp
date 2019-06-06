#include "Box3.h"
#include <Plane3.h>
using namespace CalfFluidEngine;

bool CalfFluidEngine::Box3::Intersects(const Ray3D & ray) const
{
	return bound.Intersects(transform.InverseTransformRay(ray));
}

Vector3D CalfFluidEngine::Box3::closestPointLocal(const Vector3D & otherPoint) const
{
	if (bound.Contains(otherPoint)) {
		Plane3 planes[6] = { 
			Plane3(Vector3D(1, 0, 0), bound.upperCorner),
			Plane3(Vector3D(0, 1, 0), bound.upperCorner),
			Plane3(Vector3D(0, 0, 1), bound.upperCorner),
			Plane3(Vector3D(-1, 0, 0), bound.lowerCorner),
			Plane3(Vector3D(0, -1, 0), bound.lowerCorner),
			Plane3(Vector3D(0, 0, -1), bound.lowerCorner) 
		};

		Vector3D result = planes[0].GetClosestPoint(otherPoint);
		double distanceSquared = (result - otherPoint).SquareMagnitude();

		for (int i = 1; i < 6; ++i) {
			Vector3D localResult = planes[i].GetClosestPoint(otherPoint);
			double localDistanceSquared = (localResult - otherPoint).SquareMagnitude();

			if (localDistanceSquared < distanceSquared) {
				result = localResult;
				distanceSquared = localDistanceSquared;
			}
		}

		return result;
	}
	else {

		return Vector3D::Clamp(otherPoint, bound.lowerCorner, bound.upperCorner);
	}
}

Vector3D CalfFluidEngine::Box3::closestNormalLocal(const Vector3D & otherPoint) const
{
	Plane3 planes[6] = { 
		Plane3(Vector3D(1, 0, 0), bound.upperCorner),
		Plane3(Vector3D(0, 1, 0), bound.upperCorner),
		Plane3(Vector3D(0, 0, 1), bound.upperCorner),
		Plane3(Vector3D(-1, 0, 0), bound.lowerCorner),
		Plane3(Vector3D(0, -1, 0), bound.lowerCorner),
		Plane3(Vector3D(0, 0, -1), bound.lowerCorner) 
	};

	if (bound.Contains(otherPoint)) {
		Vector3D closestNormal = planes[0].GetNormal();
		Vector3D closestPoint = planes[0].GetClosestPoint(otherPoint);
		double minDistanceSquared = (closestPoint - otherPoint).SquareMagnitude();

		for (int i = 1; i < 6; ++i) {
			Vector3D localClosestPoint = planes[i].GetClosestPoint(otherPoint);
			double localDistanceSquared =
				(localClosestPoint - otherPoint).SquareMagnitude();

			if (localDistanceSquared < minDistanceSquared) {
				closestNormal = planes[i].GetNormal();
				minDistanceSquared = localDistanceSquared;
			}
		}

		return closestNormal;
	}
	else {
		Vector3D closestPoint =
			Vector3D::Clamp(otherPoint, bound.lowerCorner, bound.upperCorner);
		Vector3D closestPointToInputPoint = otherPoint - closestPoint;
		Vector3D closestNormal = planes[0].GetNormal();
		double maxCosineAngle = Vector3D::Dot(closestNormal,closestPointToInputPoint);

		for (int i = 1; i < 6; ++i) {
			double cosineAngle = Vector3D::Dot(planes[i].GetNormal(),closestPointToInputPoint);

			if (cosineAngle > maxCosineAngle) {
				closestNormal = planes[i].GetNormal();
				maxCosineAngle = cosineAngle;
			}
		}

		return closestNormal;
	}
}

SurfaceRayIntersection3 CalfFluidEngine::Box3::closestIntersectionLocal(const Ray3D & ray) const
{
	SurfaceRayIntersection3 intersection;
	BoundingBoxRayIntersection3<double> bbRayIntersection =
		bound.GetClosestIntersection(ray);
	intersection.isIntersecting = bbRayIntersection.isIntersecting;
	if (intersection.isIntersecting) {
		intersection.distance = bbRayIntersection.tNear;
		intersection.point = ray.GetPointAt(bbRayIntersection.tNear);
		intersection.normal = GetClosestNormal(intersection.point);
	}

	return intersection;
}

BoundingBox3D CalfFluidEngine::Box3::getBoundingBoxLocal() const
{
	return bound;
}
