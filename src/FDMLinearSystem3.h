#ifndef _CalfFluidEngine_FDMLinearSystem3_
#define _CalfFluidEngine_FDMLinearSystem3_
#include <Array3.h>
namespace CalfFluidEngine {

	struct FDMMatrixRow3 {
		double center = 0.0;
		double right = 0.0;
		double up = 0.0;
		double front = 0.0;
	};

	struct FDMLinearSystem3
	{
		//Ax = b
		Array3<FDMMatrixRow3> A;
		Array3<double> x, b;
	};

	inline void MVM(const Array3<FDMMatrixRow3>& matrix, const Array3<double>& vector, Array3<double>* result)
	{
		Vector3<size_t> size = matrix.Size();
		matrix.ParallelForEach([&](size_t i, size_t j, size_t k) {
			(*result)(i, j, k) =
				matrix(i, j, k).center * vector(i, j, k) +
				((i > 0) ? matrix(i - 1, j, k).right * vector(i - 1, j, k) : 0.0) +
				((i + 1 < size.x) ? matrix(i, j, k).right * vector(i + 1, j, k) : 0.0) +
				((j > 0) ? matrix(i, j - 1, k).up * vector(i, j - 1, k) : 0.0) +
				((j + 1 < size.y) ? matrix(i, j, k).up * vector(i, j + 1, k) : 0.0) +
				((k > 0) ? matrix(i, j, k - 1).front * vector(i, j, k - 1) : 0.0) +
				((k + 1 < size.z) ? matrix(i, j, k).front * vector(i, j, k + 1) : 0.0);
		});
	}

	class IFDMLinearSystemSolver3
	{
	public:
		IFDMLinearSystemSolver3() = default;

		virtual ~IFDMLinearSystemSolver3() = default;

		//**********************************************
		//Solves the given linear system.
		//**********************************************
		virtual bool Solve(FDMLinearSystem3* system) = 0;

	};
}
#endif
