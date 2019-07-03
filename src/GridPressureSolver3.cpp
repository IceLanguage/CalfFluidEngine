#include "GridPressureSolver3.h"

CalfFluidEngine::IGridPressureSolver3::IGridPressureSolver3()
{
}

CalfFluidEngine::IGridPressureSolver3::~IGridPressureSolver3()
{
}

CalfFluidEngine::GridSinglePhasePressureSolver3::GridSinglePhasePressureSolver3()
{
	_systemSolver = std::make_shared<FDM_ICCGSolver3>(100, 1e-6);
}

CalfFluidEngine::GridSinglePhasePressureSolver3::~GridSinglePhasePressureSolver3()
{
}

void CalfFluidEngine::GridSinglePhasePressureSolver3::Solve(
	const FaceCenteredGrid3 & input, 
	double timeIntervalInSeconds, 
	FaceCenteredGrid3 * output, 
	const ScalarField3 & boundarySignedDistance)
{
	auto pos = input.GetCellCenterPosition();
	buildMarkers(input.GetResolution(), pos, boundarySignedDistance);
	buildSystem(input);

	if (_systemSolver != nullptr) {
		_systemSolver->Solve(&_system);
		applyPressureGradient(input, output);
	}
}

void CalfFluidEngine::GridSinglePhasePressureSolver3::buildMarkers(
	const Vector3<size_t>& size, 
	const std::function<Vector3D(size_t, size_t, size_t)>& pos, 
	const ScalarField3 & boundarySignedDistance)
{
	_markers.Resize(size);
	_markers.ParallelForEach([&](size_t i, size_t j, size_t k) {
		Vector3D pt = pos(i, j, k);
		if (boundarySignedDistance.Sample(pt)< 0) {
			_markers(i, j, k) = Boundary;
		}
		else {
			_markers(i, j, k) = Fluid;
		}
	});
}

void CalfFluidEngine::GridSinglePhasePressureSolver3::buildSystem(const FaceCenteredGrid3 & input)
{
	Vector3<size_t> size = input.GetResolution();
	_system.A.Resize(size);
	_system.x.Resize(size);
	_system.b.Resize(size);

	Vector3D invH = 1.0 / input.GetGridSpacing(); 
	Vector3D invHSqr = invH * invH;

	_system.A.ParallelForEach([&](size_t i, size_t j, size_t k) {
		auto& row = _system.A(i, j, k);

		// initialize
		row.center = row.right = row.up = row.front = 0.0;
		_system.b(i, j, k) = 0.0;

		if (_markers(i, j, k) == Fluid) {
			_system.b(i, j, k) = input.GetDivergenceAtCellCenter(i, j, k);

			if (i + 1 < size.x && _markers(i + 1, j, k) != Boundary) {
				row.center += invHSqr.x;
				row.right -= invHSqr.x;
			}

			if (i > 0 && _markers(i - 1, j, k) != Boundary) {
				row.center += invHSqr.x;
			}

			if (j + 1 < size.y && _markers(i, j + 1, k) != Boundary) {
				row.center += invHSqr.y;
				row.up -= invHSqr.y;
			}

			if (j > 0 && _markers(i, j - 1, k) != Boundary) {
				row.center += invHSqr.y;
			}

			if (k + 1 < size.z && _markers(i, j, k + 1) != Boundary) {
				row.center += invHSqr.z;
				row.front -= invHSqr.z;
			}

			if (k > 0 && _markers(i, j, k - 1) != Boundary) {
				row.center += invHSqr.z;
			}
		}
		else {
			row.center = 1.0;
		}
	});
}

void CalfFluidEngine::GridSinglePhasePressureSolver3::applyPressureGradient(
	const FaceCenteredGrid3 & input, FaceCenteredGrid3 * output)
{
	Vector3<size_t> size = input.GetResolution();
	auto& u = input.UArray3();
	auto& v = input.VArray3();
	auto& w = input.WArray3();
	auto& u0 = output->UArray3();
	auto& v0 = output->VArray3();
	auto& w0 = output->WArray3();
	
	const auto& x = _system.x;

	Vector3D invH = 1.0 / input.GetGridSpacing();

	x.ParallelForEach([&](size_t i, size_t j, size_t k) {
		if (_markers(i, j, k) == Fluid) {
			if (i + 1 < size.x && _markers(i + 1, j, k) != Boundary) {
				u0(i + 1, j, k) =
					u(i + 1, j, k) + invH.x * (x(i + 1, j, k) - x(i, j, k));
			}
			if (j + 1 < size.y && _markers(i, j + 1, k) != Boundary) {
				v0(i, j + 1, k) =
					v(i, j + 1, k) + invH.y * (x(i, j + 1, k) - x(i, j, k));
			}
			if (k + 1 < size.z && _markers(i, j, k + 1) != Boundary) {
				w0(i, j, k + 1) =
					w(i, j, k + 1) + invH.z * (x(i, j, k + 1) - x(i, j, k));
			}
		}
	});
}
