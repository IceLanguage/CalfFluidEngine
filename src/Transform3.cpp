#include "Transform3.h"

using namespace CalfFluidEngine;

Transform3::Transform3()
{
}

CalfFluidEngine::Transform3::Transform3(const Vector3D & translation, const Quaternion & orientation)
	:_translation(translation),_orientation(orientation)
{
}


Transform3::~Transform3()
{
}

Vector3D CalfFluidEngine::Transform3::InverseTransformPoint(const Vector3D & pointInWorld) const
{
	return _inverseOrientationMat3 * (pointInWorld - _translation);
}

Vector3D CalfFluidEngine::Transform3::InverseTransformDirection(const Vector3D & dirInWorld) const
{
	return _inverseOrientationMat3 * dirInWorld;
}

Vector3D CalfFluidEngine::Transform3::TransformPoint(const Vector3D & pointInLocal) const
{
	return (_orientationMat3 * pointInLocal) + _translation;
}

Vector3D CalfFluidEngine::Transform3::TransformDirection(const Vector3D & dirInLocal) const
{
	return _orientationMat3 * dirInLocal;
}

Ray3D CalfFluidEngine::Transform3::InverseTransformRay(const Ray3D & rayInWorld) const
{
	return Ray3D(
		InverseTransformPoint(rayInWorld.origin),
		InverseTransformDirection(rayInWorld.direction));
}
