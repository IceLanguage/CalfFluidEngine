#include "GridFluidSolver3.h"

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
}

void CalfFluidEngine::GridFluidSolver3::timeStepStart(double timeStepInSeconds)
{
}

void CalfFluidEngine::GridFluidSolver3::timeStepEnd(double timeStepInSeconds)
{
}
