#include "Collider3.h"
#include <algorithm>

using namespace CalfFluidEngine;

Collider3::Collider3()
{
}


Collider3::~Collider3()
{
}

void CalfFluidEngine::Collider3::ResolveCollision(
	double radius, 
	double restitutionCoefficient, 
	Vector3D * position, 
	Vector3D * velocity)
{
	ColliderQueryResult res;
	getClosestPoint(_surface, *position, &res);

	if (isPenetrating(res, *position, radius))
	{
		Vector3D targetNormal = res.normal;
		Vector3D targetPoint = res.point + radius * targetNormal;
		Vector3D colliderVelAtTargetPoint = res.velocity;

		Vector3D relativeVel = *velocity - colliderVelAtTargetPoint;
		double normalDotRelativeVel = Vector3D::Dot(targetNormal, relativeVel);
		Vector3D relativeVelN = normalDotRelativeVel * targetNormal;
		Vector3D relativeVelT = relativeVel - relativeVelN;

		if (normalDotRelativeVel < 0.0) {
			Vector3D deltaRelativeVelN =
				(-restitutionCoefficient - 1.0) * relativeVelN;
			relativeVelN *= -restitutionCoefficient;

			// Apply friction to the tangential component of the velocity
			// From Bridson et al., Robust Treatment of Collisions, Contact and
			// Friction for Cloth Animation, 2002
			// http://graphics.stanford.edu/papers/cloth-sig02/cloth.pdf
			if (relativeVelT.SquareMagnitude() > 0.0) {
				double frictionScale = std::max(
					1.0 - _frictionCoeffient * deltaRelativeVelN.Magnitude() /
					relativeVelT.Magnitude(),
					0.0);
				relativeVelT *= frictionScale;
			}

			*velocity =
				relativeVelN + relativeVelT + colliderVelAtTargetPoint;
		}

		*position = targetPoint;
	}
}

void CalfFluidEngine::Collider3::Update(double currentTimeInSeconds, double timeIntervalInSeconds)
{
	_surface->Update();

	if (_onUpdateCallback) {
		_onUpdateCallback(this, currentTimeInSeconds, timeIntervalInSeconds);
	}
}

void CalfFluidEngine::Collider3::getClosestPoint(
	const std::shared_ptr<Surface3>& surface, 
	const Vector3D & queryPoint, 
	ColliderQueryResult * result) const
{
	result->distance = surface->GetClosestDistance(queryPoint);
	result->point = surface->GetClosestPoint(queryPoint);
	result->normal = surface->GetClosestNormal(queryPoint);
	result->velocity = VelocityAt(queryPoint);
}

bool CalfFluidEngine::Collider3::isPenetrating(
	const ColliderQueryResult & colliderPoint, 
	const Vector3D & position, 
	double radius)
{
	return _surface->IsInside(position) ||
		colliderPoint.distance < radius;
}
