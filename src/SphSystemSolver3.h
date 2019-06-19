#ifndef _CalfFluidEngine_SphSystemSolver3_
#define _CalfFluidEngine_SphSystemSolver3_
#include <ParticleSystemSolver3.h>
#include <SphSystemData3.h>
#include <tbb\parallel_for.h>
#include <tbb\blocked_range.h>
namespace CalfFluidEngine {
	class SphSystemSolver3 : public ParticleSystemSolver3
	{
	public:
		SphSystemSolver3();
		virtual ~SphSystemSolver3();
		void SetViscosityCoefficient(
			double newViscosityCoefficient) {
			_viscosityCoefficient = std::max(newViscosityCoefficient, 0.0);
		}
		void SetPseudoViscosityCoefficient(
			double newPseudoViscosityCoefficient) {
			_pseudoViscosityCoefficient
				= std::max(newPseudoViscosityCoefficient, 0.0);
		}
		void SetTimeStepLimitScale(double newScale) {
			_timeStepLimitScale = std::max(newScale, 0.0);
		}
		std::shared_ptr<SphSystemData3> GetSphData() const;
	protected:
		virtual void accumulateForces(double timeIntervalInSeconds) override;
		virtual void onTimeStepStart(double timeStepInSeconds) override;
		virtual void onTimeStepEnd(double timeStepInSeconds) override;
		virtual unsigned int getNumberOfSubTimeSteps(
			double timeIntervalInSeconds) const override;
		virtual void accumulatePressureForce(double timeStepInSeconds);
		void accumulatePressureForce(
			const std::vector<Vector3D>& positions,
			const std::vector<double>& densities,
			const std::vector<double>& pressures,
			std::vector<Vector3D>& pressureForces);
		//! Negative pressure scaling factor.
		//! Zero means clamping. One means do nothing.
		double _negativePressureScale = 0.0;
	private:
		void accumulateViscosityForce();
		
		void computePressure();
		
		void computePseudoViscosity(double timeStepInSeconds);

		//! Exponent component of equation - of - state(or Tait's equation).
		double _eosExponent = 7.0;

		//! Speed of sound in medium to determin the stiffness of the system.
		//! Ideally, it should be the actual speed of sound in the fluid, but in
		//! practice, use lower value to trace-off performance and compressibility.
		double _speedOfSound = 100.0;

		double _viscosityCoefficient = 0.01;

		//Scales the max allowed time-step.
		double _timeStepLimitScale = 1.0;

		//! Pseudo-viscosity coefficient velocity filtering.
		//! This is a minimum "safety-net" for SPH solver which is quite
		//! sensitive to the parameters.
		double _pseudoViscosityCoefficient = 10.0;
	};
}
#endif
