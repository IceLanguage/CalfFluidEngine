#ifndef _CalfFluidEngine_GridFluidSolver3_
#define _CalfFluidEngine_GridFluidSolver3_

#include <PhysicsAnimation.h>
#include <Vector3.h>
#include <GridSystemData3.h>
#include <AdvectionSolver3.h>
#include <GridBoundaryConditionSolver3.h>
#include <GridDiffusionSolver3.h>
#include <GridPressureSolver3.h>
#include <ParticleEmitter3.h>
namespace CalfFluidEngine {
	class GridFluidSolver3 : public PhysicsAnimation
	{
	public:
		GridFluidSolver3();
		GridFluidSolver3(const Vector3<size_t>& resolution, const Vector3D& gridSpacing,
			const Vector3D& gridOrigin);
		virtual ~GridFluidSolver3();
	protected:
		virtual void onInitialize() override;
		virtual void onTimeStep(double timeIntervalInSeconds) override;
		virtual void computeExternalForces(double timeIntervalInSeconds);
		virtual void computeViscosity(double timeIntervalInSeconds);
		virtual void computePressure(double timeIntervalInSeconds);
		virtual void computeAdvection(double timeIntervalInSeconds);
		std::shared_ptr<ScalarField3> GetColliderSignedDistance() const;

		//Applies the boundary condition to the velocity field.
		void applyBoundaryCondition();

		//**********************************************
		//Returns the signed-distance representation of the fluid.
		//Positive sign area is considered to be atmosphere and won't be included
		//for computing the dynamics. By default, this will return constant scalar
		//field of -kMaxD, meaning that the entire volume is occupied with fluid.
		//**********************************************
		virtual std::shared_ptr<ScalarField3> GetFluidSignedDistance() const;

		//**********************************************
		//the function is called in GridFluidSolver3:onTimeStep(double);
		//accumulate forces
		//**********************************************
		virtual void onTimeStepStart(double timeIntervalInSeconds){}

		//**********************************************
		//the function is called in GridFluidSolver3:timeStepEnd(double);
		// Called when a time-step is about to end;
		//**********************************************
		virtual void onTimeStepEnd(double timeIntervalInSeconds) {}
	private:
		void timeStepStart(double timeStepInSeconds);
		void timeStepEnd(double timeStepInSeconds);
		void updateCollider(double timeIntervalInSeconds);
		void updateEmitter(double timeIntervalInSeconds);
		std::shared_ptr<GridSystemData3> _grids;
		std::shared_ptr<IAdvectionSolver3> _advectionSolver;
		std::shared_ptr<IGridDiffusionSolver3> _diffusionSolver;
		std::shared_ptr<IGridPressureSolver3> _pressureSolver;
		std::shared_ptr<GridBoundaryConditionSolver3> _boundaryConditionSolver;
		std::shared_ptr<Collider3> _collider;
		std::shared_ptr<ParticleEmitter3> _emitter;
		Vector3D _gravity = Vector3D(0.0, -9.8, 0.0);
		double _viscosityCoefficient = 0.0;
		double _maxCfl = 5.0;
	};
}
#endif