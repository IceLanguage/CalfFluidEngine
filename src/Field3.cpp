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
	return _value;
}

double CalfFluidEngine::ConstantVectorField3::Divergence(const Vector3D & x) const
{
	return 0.0;
}

Vector3D CalfFluidEngine::ConstantVectorField3::Curl(const Vector3D & x) const
{
	return Vector3D::zero;
}

CalfFluidEngine::ConstantScalarField3::ConstantScalarField3(double value):
_value(value)
{
}

double CalfFluidEngine::ConstantScalarField3::Sample(const Vector3D & x) const
{
	return _value;
}

std::function<double(const Vector3D&)> CalfFluidEngine::ConstantScalarField3::Sampler() const
{
	double value = _value;
	return [value](const Vector3D&) -> double {
		return value;
	};
}

Vector3D CalfFluidEngine::ConstantScalarField3::Gradient(const Vector3D & x) const
{
	return Vector3D::zero;
}

double CalfFluidEngine::ConstantScalarField3::Laplacian(const Vector3D & x) const
{
	return 0.0;
}

CustomScalarField3::CustomScalarField3(
	const std::function<double(const Vector3D&)>& customFunction,
	double derivativeResolution) :
	_customFunction(customFunction),
	_resolution(derivativeResolution)
{
}

double CustomScalarField3::Sample(const Vector3D & x) const
{
	return _customFunction(x);
}

std::function<double(const Vector3D&)> CustomScalarField3::Sampler() const
{
	return _customFunction;
}

Vector3D CustomScalarField3::Gradient(const Vector3D & x) const
{
	double left
		= _customFunction(x - Vector3D(0.5 * _resolution, 0.0, 0.0));
	double right
		= _customFunction(x + Vector3D(0.5 * _resolution, 0.0, 0.0));
	double bottom
		= _customFunction(x - Vector3D(0.0, 0.5 * _resolution, 0.0));
	double top
		= _customFunction(x + Vector3D(0.0, 0.5 * _resolution, 0.0));
	double back
		= _customFunction(x - Vector3D(0.0, 0.0, 0.5 * _resolution));
	double front
		= _customFunction(x + Vector3D(0.0, 0.0, 0.5 * _resolution));

	return Vector3D(
		(right - left) / _resolution,
		(top - bottom) / _resolution,
		(front - back) / _resolution);
}

double CustomScalarField3::Laplacian(const Vector3D & x) const
{
	double center = _customFunction(x);
	double left
		= _customFunction(x - Vector3D(0.5 * _resolution, 0.0, 0.0));
	double right
		= _customFunction(x + Vector3D(0.5 * _resolution, 0.0, 0.0));
	double bottom
		= _customFunction(x - Vector3D(0.0, 0.5 * _resolution, 0.0));
	double top
		= _customFunction(x + Vector3D(0.0, 0.5 * _resolution, 0.0));
	double back
		= _customFunction(x - Vector3D(0.0, 0.0, 0.5 * _resolution));
	double front
		= _customFunction(x + Vector3D(0.0, 0.0, 0.5 * _resolution));

	return (left + right + bottom + top + back + front - 6.0 * center)
		/ (_resolution * _resolution);
}

CalfFluidEngine::CustomVectorField3::CustomVectorField3(const std::function<Vector3D(const Vector3D&)>& customFunction, double derivativeResolution):
_customFunction(customFunction),
_resolution(derivativeResolution) {
}

Vector3D CalfFluidEngine::CustomVectorField3::Sample(const Vector3D & x) const
{
	return _customFunction(x);
}

std::function<Vector3D(const Vector3D&)> CalfFluidEngine::CustomVectorField3::Sampler() const
{
	return _customFunction;
}

double CalfFluidEngine::CustomVectorField3::Divergence(const Vector3D & x) const
{
	double left
		= _customFunction(x - Vector3D(0.5 * _resolution, 0.0, 0.0)).x;
	double right
		= _customFunction(x + Vector3D(0.5 * _resolution, 0.0, 0.0)).x;
	double bottom
		= _customFunction(x - Vector3D(0.0, 0.5 * _resolution, 0.0)).y;
	double top
		= _customFunction(x + Vector3D(0.0, 0.5 * _resolution, 0.0)).y;
	double back
		= _customFunction(x - Vector3D(0.0, 0.0, 0.5 * _resolution)).z;
	double front
		= _customFunction(x + Vector3D(0.0, 0.0, 0.5 * _resolution)).z;

	return (right - left + top - bottom + front - back) / _resolution;
}

Vector3D CalfFluidEngine::CustomVectorField3::Curl(const Vector3D & x) const
{
	Vector3D left
		= _customFunction(x - Vector3D(0.5 * _resolution, 0.0, 0.0));
	Vector3D right
		= _customFunction(x + Vector3D(0.5 * _resolution, 0.0, 0.0));
	Vector3D bottom
		= _customFunction(x - Vector3D(0.0, 0.5 * _resolution, 0.0));
	Vector3D top
		= _customFunction(x + Vector3D(0.0, 0.5 * _resolution, 0.0));
	Vector3D back
		= _customFunction(x - Vector3D(0.0, 0.0, 0.5 * _resolution));
	Vector3D front
		= _customFunction(x + Vector3D(0.0, 0.0, 0.5 * _resolution));

	double Fx_ym = bottom.x;
	double Fx_yp = top.x;
	double Fx_zm = back.x;
	double Fx_zp = front.x;

	double Fy_xm = left.y;
	double Fy_xp = right.y;
	double Fy_zm = back.y;
	double Fy_zp = front.y;

	double Fz_xm = left.z;
	double Fz_xp = right.z;
	double Fz_ym = bottom.z;
	double Fz_yp = top.z;

	return Vector3D(
		(Fz_yp - Fz_ym) - (Fy_zp - Fy_zm),
		(Fx_zp - Fx_zm) - (Fz_xp - Fz_xm),
		(Fy_xp - Fy_xm) - (Fx_yp - Fx_ym)) / _resolution;
}
