#include "AdvectionSolver3.h"

using namespace CalfFluidEngine;

CalfFluidEngine::IAdvectionSolver3::IAdvectionSolver3()
{
}

CalfFluidEngine::IAdvectionSolver3::~IAdvectionSolver3()
{
}

CalfFluidEngine::SemiLagrangianAdvectionSolver3::SemiLagrangianAdvectionSolver3()
{
}

CalfFluidEngine::SemiLagrangianAdvectionSolver3::~SemiLagrangianAdvectionSolver3()
{
}

void CalfFluidEngine::SemiLagrangianAdvectionSolver3::Advect(const FaceCenteredGrid3 & input, const VectorField3 & flow, double dt, FaceCenteredGrid3 * output, const ScalarField3 & boundarySignedDistance)
{
	auto inputSamplerFunc = input.Sampler();

	double h = min3(
		output->GetGridSpacing().x,
		output->GetGridSpacing().y,
		output->GetGridSpacing().z);

	auto uSourceDataPos = input.UPosition();
	output->UArray3().ParallelForEach([&](size_t i, size_t j, size_t k) {
		if (boundarySignedDistance.Sample(uSourceDataPos(i, j, k)) > 0.0) {

		}
	});
}
