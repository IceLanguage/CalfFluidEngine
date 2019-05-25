#include "SphSystemData3.h"
#include <SphKernel.h>
#include <tbb\parallel_for.h>
#include <tbb\blocked_range.h>
using namespace CalfFluidEngine;

SphSystemData3::SphSystemData3() : SphSystemData3(0)
{
}

CalfFluidEngine::SphSystemData3::SphSystemData3(size_t numberOfParticles)
	: ParticleSystemData3(numberOfParticles) {
	_densityIdx = AddScalarData();
	_pressureIdx = AddScalarData();

}


SphSystemData3::~SphSystemData3()
{
}

std::vector<double> CalfFluidEngine::SphSystemData3::GetDensities() const
{
	return ScalarDataAt(_densityIdx);
}

std::vector<double> CalfFluidEngine::SphSystemData3::GetPressures() const
{
	return ScalarDataAt(_pressureIdx);
}

std::vector<double> CalfFluidEngine::SphSystemData3::GetDensities()
{
	return ScalarDataAt(_densityIdx);
}

std::vector<double> CalfFluidEngine::SphSystemData3::GetPressures()
{
	return ScalarDataAt(_pressureIdx);
}

Vector3D CalfFluidEngine::SphSystemData3::Interpolate(const Vector3D & origin, const std::vector<Vector3D>& values) const
{
	Vector3D sum;
	auto d = GetDensities();
	SphStandardKernel3 kernel(_kernelRadius);
	const double m = GetParticleMass();

	GetNeighborSearcher()->ForEachNearbyPoint(
		origin, _kernelRadius, [&](size_t i, const Vector3D& neighborPosition) 
		{
			double dist = Vector3D::Distance(origin,neighborPosition);
			double weight = m / d[i] * kernel(dist);
			sum += weight * values[i];
		}
	);

	return sum;
}

void CalfFluidEngine::SphSystemData3::UpdateDensities()
{
	auto p = GetPositions();
	auto d = GetDensities();
	const double m = GetParticleMass();

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, GetNumberOfParticles()),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
		{
			double sum = SumOfKernelNearby(p[i]);
			d[i] = m * sum;
		}
	});
}

double CalfFluidEngine::SphSystemData3::SumOfKernelNearby(const Vector3D & origin) const
{
	double sum = 0.0;
	SphStandardKernel3 kernel(_kernelRadius);
	GetNeighborSearcher()->ForEachNearbyPoint(
		origin, _kernelRadius, [&](size_t, const Vector3D& neighborPosition) {
		double dist = Vector3D::Distance(origin, neighborPosition);
		sum += kernel(dist);
	});
	return sum;
}
