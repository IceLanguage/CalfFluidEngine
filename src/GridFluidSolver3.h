#ifndef _CalfFluidEngine_GridFluidSolver3_
#define _CalfFluidEngine_GridFluidSolver3_

#include <PhysicsAnimation.h>
#include <Vector3.h>
namespace CalfFluidEngine {
	class GridFluidSolver3 : public PhysicsAnimation
	{
	public:
		GridFluidSolver3();
		GridFluidSolver3(const Vector3<size_t>& resolution, const Vector3D& gridSpacing,
			const Vector3D& gridOrigin);
		virtual ~GridFluidSolver3();
	protected:
		virtual void onTimeStep(double timeIntervalInSeconds) override;
		virtual void computeExternalForces(double timeIntervalInSeconds);
		virtual void computeViscosity(double timeIntervalInSeconds);
		virtual void computePressure(double timeIntervalInSeconds);
		virtual void computeAdvection(double timeIntervalInSeconds);
	private:
		void timeStepStart(double timeStepInSeconds);
		void timeStepEnd(double timeStepInSeconds);
	};
}
#endif