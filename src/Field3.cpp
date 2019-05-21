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

ScalarField3::ScalarField3()
{
}

ScalarField3::~ScalarField3()
{
}

std::function<double(const Vector3D&)> ScalarField3::Sampler() const
{
	const ScalarField3* self = this;
	return [self](const Vector3D& x) -> double {
		return self->Sample(x);
	};
}

CalfFluidEngine::ConstantVectorField3::ConstantVectorField3(const Vector3D & value)
{
	_value = value;
}

Vector3D CalfFluidEngine::ConstantVectorField3::Sample(const Vector3D & x) const
{
	return Vector3D::zero;
}

double CalfFluidEngine::ConstantVectorField3::Divergence(const Vector3D & x) const
{
	return 0.0;
}

Vector3D CalfFluidEngine::ConstantVectorField3::Curl(const Vector3D & x) const
{
	return Vector3D::zero;
}
