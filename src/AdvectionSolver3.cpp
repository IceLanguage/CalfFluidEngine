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

void CalfFluidEngine::SemiLagrangianAdvectionSolver3::Advect(
	const ScalarGrid3 & input, 
	const VectorField3 & flow, 
	double dt, 
	ScalarGrid3 * output, 
	const ScalarField3 & boundarySignedDistance)
{
	double h = min3(
		output->GetGridSpacing().x,
		output->GetGridSpacing().y,
		output->GetGridSpacing().z);
	auto inputDataPos = input.Position();
	auto outputDataPos = output->Position();

	auto& outputData = output->GetArray3Data();
	output->ParallelForEach([&](size_t i, size_t j, size_t k) {
		if (boundarySignedDistance.Sample(inputDataPos(i, j, k)) > 0.0) {
			Vector3D pt = backTrace(
				flow, dt, h, 
				outputDataPos(i, j, k), 
				boundarySignedDistance);
			outputData(i, j, k) = input.Sampler()(pt);
		}
	});
}

void CalfFluidEngine::SemiLagrangianAdvectionSolver3::Advect(const CollocatedVectorGrid3 & input, const VectorField3 & flow, double dt, CollocatedVectorGrid3 * output, const ScalarField3 & boundarySignedDistance)
{
	double h = min3(
		output->GetGridSpacing().x,
		output->GetGridSpacing().y,
		output->GetGridSpacing().z);
	auto inputDataPos = input.Position();
	auto outputDataPos = output->Position();

	auto& outputData = output->GetArray3Data();
	output->ParallelForEach([&](size_t i, size_t j, size_t k) {
		if (boundarySignedDistance.Sample(inputDataPos(i, j, k)) > 0.0) {
			Vector3D pt = backTrace(
				flow, dt, h,
				outputDataPos(i, j, k),
				boundarySignedDistance);
			Vector3D v = input.Sample(pt);
			outputData(i, j, k) = v;
		}
	});

	
}

void CalfFluidEngine::SemiLagrangianAdvectionSolver3::Advect(
	const FaceCenteredGrid3 & input, 
	const VectorField3 & flow, 
	double dt, 
	FaceCenteredGrid3 * output, 
	const ScalarField3 & boundarySignedDistance)
{
	auto inputSamplerFunc = input.Sampler();

	double h = min3(
		output->GetGridSpacing().x,
		output->GetGridSpacing().y,
		output->GetGridSpacing().z);

	auto uSourceDataPos = input.UPosition();
	auto uTargetDataPos = output->UPosition();
	auto& uTargetData = output->UArray3();

	uTargetData.ParallelForEach([&](size_t i, size_t j, size_t k) {
		if (boundarySignedDistance.Sample(uSourceDataPos(i, j, k)) > 0.0) {
			Vector3D pt = backTrace(
				flow, dt, h, 
				uTargetDataPos(i, j, k), 
				boundarySignedDistance);
			uTargetData(i, j, k) = inputSamplerFunc(pt).x;
		}
	});

	auto vSourceDataPos = input.VPosition();
	auto vTargetDataPos = output->VPosition();
	auto& vTargetData = output->VArray3();

	vTargetData.ParallelForEach([&](size_t i, size_t j, size_t k) {
		if (boundarySignedDistance.Sample(vSourceDataPos(i, j, k)) > 0.0) {
			Vector3D pt = backTrace(
				flow, dt, h,
				vTargetDataPos(i, j, k),
				boundarySignedDistance);
			vTargetData(i, j, k) = inputSamplerFunc(pt).y;
		}
	});

	auto wSourceDataPos = input.WPosition();
	auto wTargetDataPos = output->WPosition();
	auto& wTargetData = output->WArray3();

	wTargetData.ParallelForEach([&](size_t i, size_t j, size_t k) {
		if (boundarySignedDistance.Sample(wSourceDataPos(i, j, k)) > 0.0) {
			Vector3D pt = backTrace(
				flow, dt, h,
				wTargetDataPos(i, j, k),
				boundarySignedDistance);
			wTargetData(i, j, k) = inputSamplerFunc(pt).z;
		}
	});
}

Vector3D CalfFluidEngine::SemiLagrangianAdvectionSolver3::backTrace(
	const VectorField3 & flow, 
	double dt, double h, 
	const Vector3D & startPt,
	const ScalarField3 & boundarySignedDistance)
{
	Vector3D pt0 = startPt;
	//directly method 
	/*return pt0 - dt * flow.Sample(pt0);*/

	//mid-point rule method
	Vector3D vel0 = flow.Sample(startPt);
	Vector3D midPt = startPt - 0.5 * dt * vel0; 
	Vector3D midVel = flow.Sample(midPt); 
	Vector3D pt1 = startPt - dt * midVel;

	double phi0 = boundarySignedDistance.Sample(startPt);
	double phi1 = boundarySignedDistance.Sample(pt1);

	if (phi0 * phi1 < 0.0) {
		double w = std::fabs(phi1) / (std::fabs(phi0) + std::fabs(phi1)); 
		pt1 = w * pt0 + (1.0 - w) * pt1; 
	}
	return pt1;
}
