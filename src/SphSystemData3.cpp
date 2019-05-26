#include "SphSystemData3.h"
#include <SphKernel.h>
#include <tbb\parallel_for.h>
#include <tbb\blocked_range.h>
#include <Constant.h>
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
	Vector3D sum = Vector3D::zero;
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

Vector3D CalfFluidEngine::SphSystemData3::GradientAt(size_t i, const std::vector<double>& values) const
{
	Vector3D sum;
	auto p = GetPositions();
	auto d = GetDensities();
	const auto& neighbors = GetNeighborLists()[i];
	Vector3D origin = p[i];
	SphStandardKernel3 kernel(_kernelRadius);
	const double m = GetParticleMass();

	for (size_t j : neighbors) {
		Vector3D neighborPosition = p[j];
		double dist = Vector3D::Distance(origin, neighborPosition);
		if (dist > kEpsilonD) {
			Vector3D dir = (neighborPosition - origin) / dist;
			sum += d[i] * m *
				(values[i] / (d[i] * d[i]) + values[j] / (d[j] * d[j])) *
				kernel.Gradient(dist, dir);
		}
	}

	return sum;
}

double CalfFluidEngine::SphSystemData3::LaplacianAt(size_t i, const std::vector<double>& values) const
{
	double sum = 0.0;
	auto p = GetPositions();
	auto d = GetDensities();
	const auto& neighbors = GetNeighborLists()[i];
	Vector3D origin = p[i];
	SphStandardKernel3 kernel(_kernelRadius);
	const double m = GetParticleMass();

	for (size_t j : neighbors) {
		Vector3D neighborPosition = p[j];
		double dist = Vector3D::Distance(origin, neighborPosition);
		sum += m * (values[j] - values[i]) / d[j] * kernel.Laplacian(dist);
	}

	return sum;
}