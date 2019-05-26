#include "SphKernel.h"
#include <Constant.h>
using namespace CalfFluidEngine;
CalfFluidEngine::SphStandardKernel3::SphStandardKernel3()
	:SphStandardKernel3(0)
{
}

CalfFluidEngine::SphStandardKernel3::SphStandardKernel3(double kernelRadius)
	: h(kernelRadius),h2(kernelRadius * kernelRadius), h3(h2 * h)
{
}

CalfFluidEngine::SphStandardKernel3::~SphStandardKernel3()
{
}

double CalfFluidEngine::SphStandardKernel3::operator()(double distance) const
{
	if (distance * distance >= h2) {
		return 0.0;
	}
	else {
		double x = 1.0 - distance * distance / h2;
		return 315.0 / (64.0 * kPiD * h3) * x * x * x;
	}
}

Vector3D CalfFluidEngine::SphStandardKernel3::Gradient(const Vector3D & point) const
{
	double dist = point.Magnitude();
	if (dist > kEpsilonD) {
		return Gradient(dist, point / dist);
	}
	else {
		return Vector3D::zero;
	}
}

Vector3D CalfFluidEngine::SphStandardKernel3::Gradient(double distance, const Vector3D & directionToParticle) const
{
	return -firstDerivative(distance) * directionToParticle;
}

double CalfFluidEngine::SphStandardKernel3::firstDerivative(double distance) const
{
	if (distance >= h) {
		return 0.0;
	}
	else {
		double x = 1.0 - distance * distance / h2;
		return -945.0 / (32.0 * kPiD * h3 * h2) * distance * x * x;
	}
}
