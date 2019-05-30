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
