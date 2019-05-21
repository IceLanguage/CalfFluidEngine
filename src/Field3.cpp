#include "Field3.h"

using namespace CalfFluidEngine;

Field3::Field3(){}

Field3::~Field3(){}

VectorField3::VectorField3(){}

VectorField3::~VectorField3(){}

std::function<Vector3D(const Vector3D&)> VectorField3::Sampler() const{
	const VectorField3* self = this;
	return [self](const Vector3D& x) -> Vector3D {
		return self->Sample(x);
	};
}
