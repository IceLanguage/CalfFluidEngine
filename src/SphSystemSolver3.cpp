#include "SphSystemSolver3.h"
#include <SphSystemData3.h>
#include <tbb\parallel_for.h>
#include <tbb\blocked_range.h>
#include <SphKernel.h>
#include <Constant.h>
#include <Math_utils.h>
using namespace CalfFluidEngine;

static double kTimeStepLimitBySpeedFactor = 0.4;
static double kTimeStepLimitByForceFactor = 0.25;

inline double computePressureFromEos(
	double density,
	double targetDensity,
	double eosScale,
	double eosExponent,
	double negativePressureScale) {
	// Equation of state
	// (http://www.ifi.uzh.ch/vmml/publications/pcisph/pcisph.pdf)
	double p = eosScale / eosExponent
		* (std::pow((density / targetDensity), eosExponent) - 1.0);

	// Negative pressure scaling
	if (p < 0) {
		p *= negativePressureScale;
	}

	return p;
}

SphSystemSolver3::SphSystemSolver3()
{
	setParticleSystemData(std::make_shared<SphSystemData3>());
	SetIsUsingFixedTimeSteps(false);
}


SphSystemSolver3::~SphSystemSolver3()
{
}



std::shared_ptr<SphSystemData3> CalfFluidEngine::SphSystemSolver3::GetSphData() const
{
	return std::dynamic_pointer_cast<SphSystemData3>(GetParticleSystemData());
}

void CalfFluidEngine::SphSystemSolver3::accumulateForces(double timeIntervalInSeconds)
{
	ParticleSystemSolver3::accumulateForces(timeIntervalInSeconds);
	accumulateViscosityForce();
	accumulatePressureForce(timeIntervalInSeconds);
}

void CalfFluidEngine::SphSystemSolver3::onTimeStepStart(double timeStepInSeconds)
{
	auto particles = GetSphData();

	particles->BuildNeighborSearcher(particles->GetKernelRadius());
	particles->BuildNeighborLists(particles->GetKernelRadius());
	particles->UpdateDensities();
}

void CalfFluidEngine::SphSystemSolver3::onTimeStepEnd(double timeStepInSeconds)
{
	computePseudoViscosity(timeStepInSeconds);
}

unsigned int CalfFluidEngine::SphSystemSolver3::getNumberOfSubTimeSteps(double timeIntervalInSeconds) const
{
	auto particles = GetSphData();
	size_t numberOfParticles = particles->GetNumberOfParticles();
	auto& f = particles->GetForces();

	const double kernelRadius = particles->GetKernelRadius();
	const double mass = particles->GetParticleMass();

	double maxForceMagnitude = 0.0;

	for (size_t i = 0; i < numberOfParticles; ++i) {
		maxForceMagnitude = std::max(maxForceMagnitude, f[i].Magnitude());
	}

	double timeStepLimitBySpeed
		= kTimeStepLimitBySpeedFactor * kernelRadius / _speedOfSound;
	double timeStepLimitByForce
		= kTimeStepLimitByForceFactor
		* std::sqrt(kernelRadius * mass / maxForceMagnitude);

	double desiredTimeStep
		= _timeStepLimitScale
		* std::min(timeStepLimitBySpeed, timeStepLimitByForce);

	return static_cast<unsigned int>(
		std::ceil(timeIntervalInSeconds / desiredTimeStep));
}

void CalfFluidEngine::SphSystemSolver3::accumulateViscosityForce()
{
	auto particles = GetSphData();
	size_t numberOfParticles = particles->GetNumberOfParticles();
	auto& x = particles->GetPositions();
	auto& v = particles->GetVelocities();
	auto& d = particles->GetDensities();
	auto& f = particles->GetForces();

	double mass = particles->GetParticleMass();
	const double massSquared = mass * mass;
	const SphSpikyKernel3 kernel(particles->GetKernelRadius());

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
		{
			const auto& neighbors = particles->GetNeighborLists()[i];
			for (size_t j : neighbors) {
				double dist = Vector3D::Distance(x[i],x[j]);

				f[i] += _viscosityCoefficient * massSquared
					* (v[j] - v[i]) / d[j]
					* kernel.Laplacian(dist);
			}
		}
	});
}

void CalfFluidEngine::SphSystemSolver3::accumulatePressureForce(double timeStepInSeconds)
{
	auto particles = GetSphData();
	auto& x = particles->GetPositions();
	auto& d = particles->GetDensities();
	auto& p = particles->GetPressures();
	auto& f = particles->GetForces();

	computePressure();

	accumulatePressureForce(x, d, p, f);
}

void CalfFluidEngine::SphSystemSolver3::computePressure()
{
	auto particles = GetSphData();
	size_t numberOfParticles = particles->GetNumberOfParticles();
	auto& d = particles->GetDensities();
	auto& p = particles->GetPressures();

	// See Equation 9 from
	// http://cg.informatik.uni-freiburg.de/publications/2007_SCA_SPH.pdf
	const double targetDensity = particles->GetDensity();
	const double eosScale
		= targetDensity * (_speedOfSound * _speedOfSound) / _eosExponent;

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
		{
			p[i] = computePressureFromEos(
				d[i],
				targetDensity,
				eosScale,
				_eosExponent,
				_negativePressureScale);
		}
	});
}

void CalfFluidEngine::SphSystemSolver3::accumulatePressureForce(const std::vector<Vector3D>& positions, const std::vector<double>& densities, const std::vector<double>& pressures, std::vector<Vector3D>& pressureForces)
{
	auto particles = GetSphData();
	size_t numberOfParticles = particles->GetNumberOfParticles();

	double mass = particles->GetParticleMass();
	const double massSquared = mass * mass;
	const SphSpikyKernel3 kernel(particles->GetKernelRadius());

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
		{
			const auto& neighbors = particles->GetNeighborLists()[i];
			for (size_t j : neighbors) {
				double dist = Vector3D::Distance(positions[i], positions[j]);

				if (dist > kEpsilonD) {
					Vector3D dir = (positions[j] - positions[i]) / dist;
					pressureForces[i] -= massSquared
						* (pressures[i] / (densities[i] * densities[i])
							+ pressures[j] / (densities[j] * densities[j]))
						* kernel.Gradient(dist, dir);
				}
			}
		}
	});
}

void CalfFluidEngine::SphSystemSolver3::computePseudoViscosity(double timeStepInSeconds)
{
	auto particles = GetSphData();
	size_t numberOfParticles = particles->GetNumberOfParticles();
	auto& x = particles->GetPositions();
	auto& v = particles->GetVelocities();
	auto& d = particles->GetDensities();

	const double mass = particles->GetParticleMass();
	const SphSpikyKernel3 kernel(particles->GetKernelRadius());

	std::vector<Vector3D> smoothedVelocities(numberOfParticles);

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
		{
			double weightSum = 0.0;
			Vector3D smoothedVelocity;

			const auto& neighbors = particles->GetNeighborLists()[i];
			for (size_t j : neighbors) {
				double dist = Vector3D::Distance(x[i],x[j]);
				double wj = mass / d[j] * kernel(dist);
				weightSum += wj;
				smoothedVelocity += wj * v[j];
			}

			double wi = mass / d[i];
			weightSum += wi;
			smoothedVelocity += wi * v[i];

			if (weightSum > 0.0) {
				smoothedVelocity /= weightSum;
			}

			smoothedVelocities[i] = smoothedVelocity;
		}
	});

	double factor = timeStepInSeconds * _pseudoViscosityCoefficient;
	factor = Clamp(factor, 0.0, 1.0); 

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
		{
			v[i] = Lerp(
				v[i], smoothedVelocities[i], factor);
		}
	});
}
