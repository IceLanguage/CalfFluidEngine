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
	public:
		//Ax = b
		Array3<FDMMatrixRow3> A;
		Array3<double> x, b;

		static double Dot(const Array3<double>& a, const Array3<double>& b) {
			Vector3<size_t> size = a.Size();

			double result = 0.0;

			a.ParallelForEach([&](size_t i, size_t j, size_t k) {
				result += a(i, j, k) * b(i, j, k);
			});

			return result;
		}
	};

	// res = m * v
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

	// res = A - bx
	inline void Residual(
		const Array3<FDMMatrixRow3>& A, 
		const Array3<double>& x, 
		const Array3<double>& b, 
		Array3<double>* result)
	{
		Vector3<size_t> size = A.Size();
		A.ParallelForEach([&](size_t i, size_t j, size_t k) {
			(*result)(i, j, k) =
				b(i, j, k) - A(i, j, k).center * x(i, j, k) -
				((i > 0) ? A(i - 1, j, k).right * x(i - 1, j, k) : 0.0) -
				((i + 1 < size.x) ? A(i, j, k).right * x(i + 1, j, k) : 0.0) -
				((j > 0) ? A(i, j - 1, k).up * x(i, j - 1, k) : 0.0) -
				((j + 1 < size.y) ? A(i, j, k).up * x(i, j + 1, k) : 0.0) -
				((k > 0) ? A(i, j, k - 1).front * x(i, j, k - 1) : 0.0) -
				((k + 1 < size.z) ? A(i, j, k).front * x(i, j, k + 1) : 0.0);
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

	class FDMJacobiSolver3 final : public IFDMLinearSystemSolver3 {
	public:
		virtual bool Solve(FDMLinearSystem3* system) override;
	private:

		// formula in markdown: $ x^{(k+1)}_i  = \frac{1}{a_{ii}} \left(b_i -\sum_{j\ne i}a_{ij}x^{(k)}_j\right),\quad i=1,2,\ldots,n.$
		void relax(
			const Array3<FDMMatrixRow3>& A, 
			const Array3<double>& b, 
			Array3<double>* x,
			Array3<double>* xTemp);

		unsigned int _maxNumberOfIterations;
		unsigned int _residualCheckInterval;
		double _maxResidualTolerance;
		Array3<double> _xTemp;
		Array3<double> _residual;
	};

	//Gauss Seidel method is programmatically consistent with the Jacobi method 
	typename FDMJacobiSolver3 FDMGaussSeidelSolver3;
}
#endif
