#include "GridPressureSolver3.h"

double FractionInsideSignedDistance(double phi0, double phi1) {
	if (phi0 < 0 && phi1 < 0) {
		return 1;
	}
	else if (phi0 < 0 && phi1 >= 0) {
		return phi0 / (phi0 - phi1);
	}
	else if (phi0 >= 0 && phi1< 0) {
		return phi1 / (phi1 - phi0);
	}
	else {
		return 0;
	}
}

void cycleArray(double* arr, int size) {
	double t = arr[0];
	for (int i = 0; i < size - 1; ++i) arr[i] = arr[i + 1];
	arr[size - 1] = t;
}

double FractionInsideSignedDistance(
	double phiBottomLeft, 
	double phiBottomRight, 
	double phiTopLeft,
	double phiTopRight) {
	int inside_count = (phiBottomLeft < 0 ? 1 : 0) + (phiTopLeft < 0 ? 1 : 0) +
		(phiBottomRight < 0 ? 1 : 0) + (phiTopRight < 0 ? 1 : 0);
	double list[] = { phiBottomLeft, phiBottomRight, phiTopRight, phiTopLeft };

	if (inside_count == 4) {
		return 1;
	}
	else if (inside_count == 3) {
		// rotate until the positive value is in the first position
		while (list[0] < 0) {
			cycleArray(list, 4);
		}

		// Work out the area of the exterior triangle
		double side0 = 1 - FractionInsideSignedDistance(list[0], list[3]);
		double side1 = 1 - FractionInsideSignedDistance(list[0], list[1]);
		return 1 - 0.5f * side0 * side1;
	}
	else if (inside_count == 2) {
		// rotate until a negative value is in the first position, and the next
		// negative is in either slot 1 or 2.
		while (list[0] >= 0 || !(list[1] < 0 || list[2] < 0)) {
			cycleArray(list, 4);
		}

		if (list[1] < 0) {  // the matching signs are adjacent
			double side_left = FractionInsideSignedDistance(list[0], list[3]);
			double side_right = FractionInsideSignedDistance(list[1], list[2]);
			return 0.5f * (side_left + side_right);
		}
		else {  // matching signs are diagonally opposite
				// determine the centre point's sign to disambiguate this case
			double middle_point = 0.25f * (list[0] + list[1] + list[2] + list[3]);
			if (middle_point < 0) {
				double area = 0;

				// first triangle (top left)
				double side1 = 1 - FractionInsideSignedDistance(list[0], list[3]);
				double side3 = 1 - FractionInsideSignedDistance(list[2], list[3]);

				area += 0.5f * side1 * side3;

				// second triangle (top right)
				double side2 = 1 - FractionInsideSignedDistance(list[2], list[1]);
				double side0 = 1 - FractionInsideSignedDistance(list[0], list[1]);
				area += 0.5f * side0 * side2;

				return 1 - area;
			}
			else {
				double area = 0;

				// first triangle (bottom left)
				double side0 = FractionInsideSignedDistance(list[0], list[1]);
				double side1 = FractionInsideSignedDistance(list[0], list[3]);
				area += 0.5f * side0 * side1;

				// second triangle (top right)
				double side2 = FractionInsideSignedDistance(list[2], list[1]);
				double side3 = FractionInsideSignedDistance(list[2], list[3]);
				area += 0.5f * side2 * side3;
				return area;
			}
		}
	}
	else if (inside_count == 1) {
		// rotate until the negative value is in the first position
		while (list[0] >= 0) {
			cycleArray(list, 4);
		}

		// Work out the area of the interior triangle, and subtract from 1.
		double side0 = FractionInsideSignedDistance(list[0], list[3]);
		double side1 = FractionInsideSignedDistance(list[0], list[1]);
		return 0.5f * side0 * side1;
	}
	else {
		return 0;
	}
}
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

CalfFluidEngine::GridFractionalSinglePhasePressureSolver3::GridFractionalSinglePhasePressureSolver3()
{
	_systemSolver = std::make_shared<FDM_ICCGSolver3>(100, 1e-6);
}

CalfFluidEngine::GridFractionalSinglePhasePressureSolver3::~GridFractionalSinglePhasePressureSolver3()
{
}

void CalfFluidEngine::GridFractionalSinglePhasePressureSolver3::Solve(
	const FaceCenteredGrid3 & input, 
	double timeIntervalInSeconds, 
	FaceCenteredGrid3 * output, 
	const ScalarField3 & boundarySignedDistance)
{
	buildWeights(input, boundarySignedDistance);
	buildSystem(input);

	if (_systemSolver != nullptr) {
		_systemSolver->Solve(&_system);
		applyPressureGradient(input, output);
	}
}

void CalfFluidEngine::GridFractionalSinglePhasePressureSolver3::buildWeights(
	const FaceCenteredGrid3 & input, 
	const ScalarField3 & boundarySignedDistance)
{
	auto uPos = input.UPosition();
	auto vPos = input.VPosition();
	auto wPos = input.WPosition();
	Vector3D h = input.GetGridSpacing();
	_uWeights.ParallelForEach([&](size_t i, size_t j, size_t k) {
		Vector3D pt = uPos(i, j, k);
		double phi0 =
			boundarySignedDistance.Sample(pt + Vector3D(0.0, -0.5 * h.y, -0.5 * h.z));
		double phi1 =
			boundarySignedDistance.Sample(pt + Vector3D(0.0, 0.5 * h.y, -0.5 * h.z));
		double phi2 =
			boundarySignedDistance.Sample(pt + Vector3D(0.0, -0.5 * h.y, 0.5 * h.z));
		double phi3 =
			boundarySignedDistance.Sample(pt + Vector3D(0.0, 0.5 * h.y, 0.5 * h.z));
		double frac = FractionInsideSignedDistance(phi0, phi1, phi2, phi3);
		double weight = Clamp(1.0 - frac, 0.0, 1.0);

		if (weight < 0.01 && weight > 0.0) {
			weight = 0.01;
		}

		_uWeights(i, j, k) = static_cast<float>(weight);
	});
}

void CalfFluidEngine::GridFractionalSinglePhasePressureSolver3::buildSystem(const FaceCenteredGrid3 & input)
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

		double temp;
		if (i + 1 < size.x) {
			temp = _uWeights(i + 1, j, k) * invHSqr.x;
			row.center += temp;
			row.right -= temp;
			_system.b(i, j, k)
				+= temp * input.u(i + 1, j, k);
		}
		else {
			_system.b(i, j, k)
				+= input.u(i + 1, j, k) * invH.x;
		}

		if (i > 0) {
			temp = _uWeights(i, j, k) * invHSqr.x;
			row.center += temp;
			_system.b(i, j, k)
				-= temp * input.u(i, j, k);
		}
		else {
			_system.b(i, j, k)
				-= input.u(i, j, k) * invH.x;
		}

		if (j + 1 < size.y) {
			temp = _vWeights(i, j + 1, k) * invHSqr.y;
			row.center += temp;
			row.up -= temp;
			_system.b(i, j, k)
				+= temp * input.v(i, j + 1, k) ;
		}
		else {
			_system.b(i, j, k)
				+= input.v(i, j + 1, k) * invH.y;
		}

		if (j > 0 ) {
			temp = _uWeights(i, j, k) * invHSqr.y;
			row.center += temp;
			_system.b(i, j, k)
				-= temp * input.v(i, j, k);
		}
		else {
			_system.b(i, j, k)
				-= input.v(i, j, k) * invH.y;
		}

		if (k + 1 < size.z) {

			temp = _wWeights(i, j, k + 1) * invHSqr.z;
			row.center += temp;
			row.front -= temp;
			_system.b(i, j, k)
				+= temp * input.w(i, j, k + 1);
		}
		else {
			_system.b(i, j, k)
				+= input.w(i, j + 1, k) * invH.z;
		}

		if (k > 0) {
			temp = _wWeights(i, j, k) * invHSqr.z;
			row.center += temp;
			_system.b(i, j, k)
				-= temp * input.w(i, j, k);
		}
		else {
			_system.b(i, j, k)
				-= input.w(i, j, k) * invH.z;
		}
	});
}

void CalfFluidEngine::GridFractionalSinglePhasePressureSolver3::applyPressureGradient(const FaceCenteredGrid3 & input, FaceCenteredGrid3 * output)
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
		if (i + 1 < size.x && _uWeights(i + 1, j, k) > 0.0) {
			u0(i + 1, j, k) =
				u(i + 1, j, k) + invH.x * (x(i + 1, j, k) - x(i, j, k));
		}
		if (j + 1 < size.y && _vWeights(i, j + 1, k) > 0.0) {
			v0(i, j + 1, k) =
				v(i, j + 1, k) + invH.y * (x(i, j + 1, k) - x(i, j, k));
		}
		if (k + 1 < size.z && _wWeights(i, j, k + 1) > 0.0) {
			w0(i, j, k + 1) =
				w(i, j, k + 1) + invH.z * (x(i, j, k + 1) - x(i, j, k));
		}
	});
}
