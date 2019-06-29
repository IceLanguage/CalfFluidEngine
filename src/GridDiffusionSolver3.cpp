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
	Vector3D h = source.GetGridSpacing();
	Vector3D c = timeIntervalInSeconds * diffusionCoefficient / (h * h);

	buildMarkers(source.GetDataSize(), source.Position(), boundarySignedDistance, fluidSignedDistance);
	buildMatrix(source.GetDataSize(), c);
	buildVectors(source.GetArray3Data(), c);

	if (_systemSolver != nullptr) {
		_systemSolver->Solve(&_system);

		source.ParallelForEach(
			[&](size_t i, size_t j, size_t k) {
			(*dest)(i, j, k) = _system.x(i, j, k);
		});
	}
}

void CalfFluidEngine::GridBackwardEulerDiffusionSolver3::buildMarkers(
	const Vector3<size_t>& size, 
	const std::function<Vector3D(size_t, size_t, size_t)>& pos,
	const ScalarField3 & boundarySignedDistance, 
	const ScalarField3 & fluidSignedDistance)
{
	_markers.Resize(size);

	_markers.ParallelForEach([&](size_t i, size_t j, size_t k) {
		if (boundarySignedDistance.Sample(pos(i, j, k)) < 0) {
			_markers(i, j, k) = Boundary;
		}
		else if (fluidSignedDistance.Sample(pos(i, j, k))< 0) {
			_markers(i, j, k) = Fluid;
		}
	});
}

void CalfFluidEngine::GridBackwardEulerDiffusionSolver3::buildMatrix(
	const Vector3<size_t>& size, 
	const Vector3D & c)
{
	_system.A.ParallelForEach([&](size_t i, size_t j, size_t k)
	{
		auto& row = _system.A(i, j, k);

		// Initialize
		row.center = 1.0;
		row.right = row.up = row.front = 0.0;

		if (_markers(i, j, k) != Fluid)  return;

		if (i + 1 < size.x && _markers(i + 1, j, k) == Fluid) {
			row.center += c.x;
			row.right -= c.x;
		}
		if (i > 0 && _markers(i - 1, j, k) == Fluid)
		{
			row.center += c.x;
		}
		if (j + 1 < size.y && _markers(i, j + 1, k) == Fluid) {
			row.center += c.y;
			row.up -= c.y;
		}
		if (j > 0 && _markers(i, j - 1, k) == Fluid)
		{
			row.center += c.y;
		}
		if (k + 1 < size.z && _markers(i, j, k + 1) == Fluid)
		{
			row.center += c.z;
			row.front -= c.z;
		}
		if (k > 0 && _markers(i, j, k - 1) == Fluid)
		{
			row.center += c.z;
		}
	});
}

void CalfFluidEngine::GridBackwardEulerDiffusionSolver3::buildVectors(const Array3<double>& f, const Vector3D & c)
{
	Vector3<size_t> size = f.Size();
	_system.x.Resize(size, 0.0);
	_system.b.Resize(size, 0.0);

	_system.x.ParallelForEach(
		[&](size_t i, size_t j, size_t k) {
		_system.b(i, j, k) = _system.x(i, j, k) = f(i, j, k);

		if (_markers(i, j, k) != Fluid) return;

		if (i + 1 < size.x && _markers(i + 1, j, k) == Boundary) {
			_system.b(i, j, k) += c.x * f(i + 1, j, k);
		}

		if (i > 0 && _markers(i - 1, j, k) == Boundary) {
			_system.b(i, j, k) += c.x * f(i - 1, j, k);
		}

		if (j + 1 < size.y && _markers(i, j + 1, k) == Boundary) {
			_system.b(i, j, k) += c.y * f(i, j + 1, k);
		}

		if (j > 0 && _markers(i, j - 1, k) == Boundary) {
			_system.b(i, j, k) += c.y * f(i, j - 1, k);
		}

		if (k + 1 < size.z && _markers(i, j, k + 1) == Boundary) {
			_system.b(i, j, k) += c.z * f(i, j, k + 1);
		}

		if (k > 0 && _markers(i, j, k - 1) == Boundary) {
			_system.b(i, j, k) += c.z * f(i, j, k - 1);
		}
	});
}
