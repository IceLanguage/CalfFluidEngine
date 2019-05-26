#ifndef _CalfFluidEngine_SphSystemSolver3_
#define _CalfFluidEngine_SphSystemSolver3_
#include <ParticleSystemSolver3.h>
namespace CalfFluidEngine {
	class SphSystemSolver3 : public ParticleSystemSolver3
	{
	public:
		SphSystemSolver3();
		virtual ~SphSystemSolver3();
	protected:
		virtual void accumulateForces(double timeIntervalInSeconds) override;
		virtual void onTimeStepStart(double timeStepInSeconds) override;
		virtual void onTimeStepEnd(double timeStepInSeconds) override;
	private:
		void accumulateViscosityForce();
		void accumulatePressureForce(double timeStepInSeconds);
		void computePressure();
		void accumulatePressureForce(
			const std::vector<Vector3D>& positions,
			const std::vector<double>& densities,
			const std::vector<double>& pressures,
			std::vector<Vector3D> pressureForces);
		void computePseudoViscosity(double timeStepInSeconds);
		double _kernelRadius;
	};
}
#endif
