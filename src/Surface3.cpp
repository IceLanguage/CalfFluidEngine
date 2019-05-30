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
	return Vector3D();
}

Vector3D CalfFluidEngine::Surface3::GetClosestNormal(const Vector3D & otherPoint) const
{
	return Vector3D();
}

double CalfFluidEngine::Surface3::GetClosestDistance(const Vector3D & otherPoint) const
{
	return 0.0;
}
