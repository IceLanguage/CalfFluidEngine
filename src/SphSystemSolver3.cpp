#include "SphSystemSolver3.h"
#include <SphSystemData3.h>
using namespace CalfFluidEngine;

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

	particles->BuildNeighborSearcher(_kernelRadius);
	particles->BuildNeighborLists(_kernelRadius);
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
}

void CalfFluidEngine::SphSystemSolver3::accumulatePressureForce(const std::vector<Vector3D>& positions, const std::vector<double>& densities, const std::vector<double>& pressures, std::vector<Vector3D> pressureForces)
{
}

void CalfFluidEngine::SphSystemSolver3::computePseudoViscosity(double timeStepInSeconds)
{
}
