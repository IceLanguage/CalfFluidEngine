#include "SphSystemSolver3.h"
#include <SphSystemData3.h>
#include <tbb\parallel_for.h>
#include <tbb\blocked_range.h>
using namespace CalfFluidEngine;

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

void CalfFluidEngine::SphSystemSolver3::accumulateForces(double timeIntervalInSeconds)
{
	ParticleSystemSolver3::accumulateForces(timeIntervalInSeconds);
	accumulateViscosityForce();
	accumulatePressureForce(timeIntervalInSeconds);
}

void CalfFluidEngine::SphSystemSolver3::onTimeStepStart(double timeStepInSeconds)
{
	auto particles = std::dynamic_pointer_cast<SphSystemData3>(GetParticleSystemData());

	particles->BuildNeighborSearcher(particles->GetKernelRadius());
	particles->BuildNeighborLists(particles->GetKernelRadius());
	particles->UpdateDensities();
}

void CalfFluidEngine::SphSystemSolver3::onTimeStepEnd(double timeStepInSeconds)
{
	computePseudoViscosity(timeStepInSeconds);
}

void CalfFluidEngine::SphSystemSolver3::accumulateViscosityForce()
{
	
}

void CalfFluidEngine::SphSystemSolver3::accumulatePressureForce(double timeStepInSeconds)
{
	auto particles = std::dynamic_pointer_cast<SphSystemData3>(GetParticleSystemData());
	auto x = particles->GetPositions();
	auto d = particles->GetDensities();
	auto p = particles->GetPressures();
	auto f = particles->GetForces();

	computePressure();

	accumulatePressureForce(x, d, p, f);
}

void CalfFluidEngine::SphSystemSolver3::computePressure()
{
	auto particles = std::dynamic_pointer_cast<SphSystemData3>(GetParticleSystemData());
	size_t numberOfParticles = particles->GetNumberOfParticles();
	auto d = particles->GetDensities();
	auto p = particles->GetPressures();

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

void CalfFluidEngine::SphSystemSolver3::accumulatePressureForce(const std::vector<Vector3D>& positions, const std::vector<double>& densities, const std::vector<double>& pressures, std::vector<Vector3D> pressureForces)
{
}

void CalfFluidEngine::SphSystemSolver3::computePseudoViscosity(double timeStepInSeconds)
{
}
