#ifndef _CalfFluidEngine_GridDiffusionSolver3_
#define _CalfFluidEngine_GridDiffusionSolver3_
#include <Grid3.h>
#include <Constant.h>
namespace CalfFluidEngine {
	class IGridDiffusionSolver3
	{
	public:
		IGridDiffusionSolver3();
		virtual ~IGridDiffusionSolver3();

		//**********************************************
		//Solves diffusion equation for a scalar field.
		//**********************************************
		virtual void Solve(
			const ScalarGrid3& source,
			double diffusionCoefficient,
			double timeIntervalInSeconds,
			ScalarGrid3* dest,
			const ScalarField3& boundarySignedDistance = ConstantScalarField3(kMaxD),
			const ScalarField3& fluidSignedDistance = ConstantScalarField3(-kMaxD)) = 0;

		//**********************************************
		//Solves diffusion equation for a collocated vector field.
		//**********************************************
		virtual void Solve(
			const CollocatedVectorGrid3& source,
			double diffusionCoefficient,
			double timeIntervalInSeconds,
			CollocatedVectorGrid3* dest,
			const ScalarField3& boundarySignedDistance = ConstantScalarField3(kMaxD),
			const ScalarField3& fluidSignedDistance = ConstantScalarField3(-kMaxD)) = 0;

		//**********************************************
		//Solves diffusion equation for a face-centered vector field.
		//**********************************************
		virtual void Solve(
			const FaceCenteredGrid3& source,
			double diffusionCoefficient,
			double timeIntervalInSeconds,
			FaceCenteredGrid3* dest,
			const ScalarField3& boundarySignedDistance = ConstantScalarField3(kMaxD),
			const ScalarField3& fluidSignedDistance = ConstantScalarField3(-kMaxD)) = 0;
	};

	class GridForwardEulerDiffusionSolver3 final : public IGridDiffusionSolver3 {
	public:
		void Solve(
			const ScalarGrid3& source,
			double diffusionCoefficient,
			double timeIntervalInSeconds,
			ScalarGrid3* dest,
			const ScalarField3& boundarySignedDistance,
			const ScalarField3& fluidSignedDistance) override;
	};

	class GridBackwardEulerDiffusionSolver3 final :public IGridDiffusionSolver3 {
	public:
		void Solve(
			const ScalarGrid3& source,
			double diffusionCoefficient,
			double timeIntervalInSeconds,
			ScalarGrid3* dest,
			const ScalarField3& boundarySignedDistance,
			const ScalarField3& fluidSignedDistance) override;
	};
}
#endif