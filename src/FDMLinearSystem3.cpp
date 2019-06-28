#include "FDMLinearSystem3.h"

using namespace CalfFluidEngine;

bool CalfFluidEngine::FDMJacobiSolver3::Solve(FDMLinearSystem3 * system)
{
	_xTemp.Resize(system->x.Size());
	_residual.Resize(system->x.Size());

	unsigned int lastNumberOfIterations = _maxNumberOfIterations;

	for (unsigned int iter = 0; iter < _maxNumberOfIterations; ++iter) {

		relax(system->A, system->b, &system->x, &_xTemp);
		_xTemp.Swap(system->x);

		if (iter != 0 && iter % _residualCheckInterval == 0) {

			Residual(system->A, system->x, system->b, &_residual);

			if (std::sqrt(FDMLinearSystem3::Dot(_residual, _residual)) < _maxResidualTolerance)
			{
				lastNumberOfIterations = iter + 1;
				break;
			}
		}
	}
	Residual(system->A, system->x, system->b, &_residual);

	return std::sqrt(FDMLinearSystem3::Dot(_residual, _residual)) < _maxResidualTolerance;
}

void CalfFluidEngine::FDMJacobiSolver3::relax(
	const Array3<FDMMatrixRow3>& R, 
	const Array3<double>& b, 
	Array3<double>* x_, 
	Array3<double>* xTemp_)
{
	Vector3<size_t> size = R.Size();
	Array3<double>& x = *x_;
	Array3<double>& xTemp = *xTemp_;

	R.ParallelForEach([&](size_t i, size_t j, size_t k) {
		double r =
			((i > 0) ? R(i - 1, j, k).right * x(i - 1, j, k) : 0.0) +
			((i + 1 < size.x) ? R(i, j, k).right * x(i + 1, j, k) : 0.0) +
			((j > 0) ? R(i, j - 1, k).up * x(i, j - 1, k) : 0.0) +
			((j + 1 < size.y) ? R(i, j, k).up * x(i, j + 1, k) : 0.0) +
			((k > 0) ? R(i, j, k - 1).front * x(i, j, k - 1) : 0.0) +
			((k + 1 < size.z) ? R(i, j, k).front * x(i, j, k + 1) : 0.0);

		xTemp(i, j, k) = (b(i, j, k) - r) / R(i, j, k).center;
	});
}
