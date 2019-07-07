#ifndef _CalfFluidEngine_GridPressureSolver3_
#define _CalfFluidEngine_GridPressureSolver3_
#include <Grid3.h>
#include <Constant.h>
#include <FDMLinearSystem3.h>
namespace CalfFluidEngine {
	

	class IGridPressureSolver3 {
	public:
		IGridPressureSolver3();

		virtual ~IGridPressureSolver3();

		//**********************************************
		//Solves the pressure term and apply it to the velocity field.
		//**********************************************
		virtual void Solve(
			const FaceCenteredGrid3& input, double timeIntervalInSeconds,
			FaceCenteredGrid3* output,
			const ScalarField3& boundarySignedDistance = ConstantScalarField3(kMaxD)) = 0;
	};

	class GridSinglePhasePressureSolver3 : public IGridPressureSolver3 {
	public:
		GridSinglePhasePressureSolver3();
		virtual ~GridSinglePhasePressureSolver3();

		virtual void Solve(
			const FaceCenteredGrid3& input, double timeIntervalInSeconds,
			FaceCenteredGrid3* output,
			const ScalarField3& boundarySignedDistance) override;
	private:
		enum Marker {
			Fluid = 0,
			Boundary = 1
		};
		void buildMarkers(
			const Vector3<size_t>& size,
			const std::function<Vector3D(size_t, size_t, size_t)>& pos,
			const ScalarField3& boundarySignedDistance);
		void buildSystem(const FaceCenteredGrid3& input);
		void applyPressureGradient(const FaceCenteredGrid3& input,
			FaceCenteredGrid3* output);
		FDMLinearSystem3 _system;
		std::shared_ptr<IFDMLinearSystemSolver3> _systemSolver;
		Array3<char> _markers;
	};

	class GridFractionalSinglePhasePressureSolver3 : public IGridPressureSolver3 {
	public:
		GridFractionalSinglePhasePressureSolver3();
		virtual ~GridFractionalSinglePhasePressureSolver3();

		virtual void Solve(
			const FaceCenteredGrid3& input, double timeIntervalInSeconds,
			FaceCenteredGrid3* output,
			const ScalarField3& boundarySignedDistance) override;
	private:
		void buildWeights(const FaceCenteredGrid3& input,
			const ScalarField3& boundarySignedDistance);
		void buildSystem(const FaceCenteredGrid3& input);
		void applyPressureGradient(const FaceCenteredGrid3& input,
			FaceCenteredGrid3* output);
		std::shared_ptr<IFDMLinearSystemSolver3> _systemSolver;
		FDMLinearSystem3 _system;
		Array3<float> _uWeights;
		Array3<float> _vWeights;
		Array3<float> _wWeights;
	};
}
#endif
