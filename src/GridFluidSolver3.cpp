#include "GridFluidSolver3.h"
#include <memory>
CalfFluidEngine::GridFluidSolver3::GridFluidSolver3()
	: GridFluidSolver3({ 1, 1, 1 }, { 1, 1, 1 }, { 0, 0, 0 })
{
}

CalfFluidEngine::GridFluidSolver3::GridFluidSolver3(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & gridOrigin)
{
}

CalfFluidEngine::GridFluidSolver3::~GridFluidSolver3()
{
}

void CalfFluidEngine::GridFluidSolver3::onTimeStep(double timeIntervalInSeconds)
{
	timeStepStart(timeIntervalInSeconds);
	computeExternalForces(timeIntervalInSeconds);
	computeViscosity(timeIntervalInSeconds);
	computePressure(timeIntervalInSeconds);
	computeAdvection(timeIntervalInSeconds);
	timeStepEnd(timeIntervalInSeconds);
}

void CalfFluidEngine::GridFluidSolver3::computeExternalForces(double timeIntervalInSeconds)
{
}

void CalfFluidEngine::GridFluidSolver3::computeViscosity(double timeIntervalInSeconds)
{
}

void CalfFluidEngine::GridFluidSolver3::computePressure(double timeIntervalInSeconds)
{
}

void CalfFluidEngine::GridFluidSolver3::computeAdvection(double timeIntervalInSeconds)
{
	std::shared_ptr<FaceCenteredGrid3> vel = _grids->GetVelocity();
	if (_advectionSolver != nullptr) 
	{
		auto vel0 = std::dynamic_pointer_cast<FaceCenteredGrid3>(vel->Clone());
		_advectionSolver->Advect(*vel0, *vel0, timeIntervalInSeconds, vel.get(),
			*GetColliderSignedDistance());
	}
}

std::shared_ptr<ScalarField3> CalfFluidEngine::GridFluidSolver3::GetColliderSignedDistance() const
{
	return _boundaryConditionSolver->GetColliderSignedDistance();
}

void CalfFluidEngine::GridFluidSolver3::timeStepStart(double timeStepInSeconds)
{
}

void CalfFluidEngine::GridFluidSolver3::timeStepEnd(double timeStepInSeconds)
{
}
