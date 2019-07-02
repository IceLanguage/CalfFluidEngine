#ifndef _CalfFluidEngine_AdvectionSolver3_
#define _CalfFluidEngine_AdvectionSolver3_
#include <Grid3.h>
#include <Constant.h>
namespace CalfFluidEngine {
	class IAdvectionSolver3
	{
	public:
		IAdvectionSolver3();
		virtual ~IAdvectionSolver3();

		//**********************************************
		//Solves advection equation for given scalar grid.
		//**********************************************
		virtual void Advect(
			const ScalarGrid3& input,
			const VectorField3& flow,
			double dt,
			ScalarGrid3* output,
			const ScalarField3& boundarySignedDistance 
			= ConstantScalarField3(kMaxD)) = 0;

		//**********************************************
		//Solves advection equation for given collocated vector grid.
		//**********************************************
		virtual void Advect(
			const CollocatedVectorGrid3& input,
			const VectorField3& flow,
			double dt,
			CollocatedVectorGrid3* output,
			const ScalarField3& boundarySignedDistance
			= ConstantScalarField3(kMaxD)) = 0;

		//**********************************************
		//Solves advection equation for given face-centered vector grid.
		//**********************************************
		virtual void Advect(
			const FaceCenteredGrid3& input,
			const VectorField3& flow,
			double dt,
			FaceCenteredGrid3* output,
			const ScalarField3& boundarySignedDistance
			= ConstantScalarField3(kMaxD)) = 0;
	};

	class SemiLagrangianAdvectionSolver3 : public IAdvectionSolver3
	{
	public:
		SemiLagrangianAdvectionSolver3();
		virtual ~SemiLagrangianAdvectionSolver3();

		void Advect(
			const ScalarGrid3& input,
			const VectorField3& flow,
			double dt,
			ScalarGrid3* output,
			const ScalarField3& boundarySignedDistance) override final;
		void Advect(
				const CollocatedVectorGrid3& input,
				const VectorField3& flow,
				double dt,
				CollocatedVectorGrid3* output,
				const ScalarField3& boundarySignedDistance) override final;
		void Advect(
			const FaceCenteredGrid3& input, 
			const VectorField3& flow,
			double dt, 
			FaceCenteredGrid3* output,
			const ScalarField3& boundarySignedDistance) override final;
	private:
		Vector3D backTrace(
			const VectorField3& flow, 
			double dt, double h,
			const Vector3D& pt0, 
			const ScalarField3& boundarySignedDistance);
	};
}
#endif