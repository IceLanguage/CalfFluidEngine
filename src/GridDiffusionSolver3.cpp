#include "GridDiffusionSolver3.h"

using namespace CalfFluidEngine;

CalfFluidEngine::IGridDiffusionSolver3::IGridDiffusionSolver3()
{
}

CalfFluidEngine::IGridDiffusionSolver3::~IGridDiffusionSolver3()
{
}

void CalfFluidEngine::GridForwardEulerDiffusionSolver3::Solve(
	const ScalarGrid3 & source, 
	double diffusionCoefficient, 
	double timeIntervalInSeconds, 
	ScalarGrid3 * dest, 
	const ScalarField3 & boundarySignedDistance,
	const ScalarField3 & fluidSignedDistance)
{
	source.ParallelForEach(
		[&](size_t i, size_t j, size_t k) {
		(*dest)(i, j, k)
			= source(i, j, k)
			+ diffusionCoefficient
			* timeIntervalInSeconds
			* source.GetLaplacianAtDataPoint(i, j, k);
	});
}

void CalfFluidEngine::GridBackwardEulerDiffusionSolver3::Solve(
	const ScalarGrid3 & source, 
	double diffusionCoefficient, 
	double timeIntervalInSeconds, 
	ScalarGrid3 * dest, 
	const ScalarField3 & boundarySignedDistance, 
	const ScalarField3 & fluidSignedDistance)
{
}
