#ifndef _CalfFluidEngine_GridPressureSolver3_
#define _CalfFluidEngine_GridPressureSolver3_
#include <Grid3.h>
#include <Constant.h>
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
			const ScalarField3& boundarySdf = ConstantScalarField3(kMaxD)) = 0;
	};
}
#endif
