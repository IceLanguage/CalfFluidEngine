#ifndef _CalfFluidEngine_ParticleSystemSolver3_
#define _CalfFluidEngine_ParticleSystemSolver3_

#include <PhysicsAnimation.h>
#include <vector>
#include <Vector3.h>
#include <memory>
#include <Collider3.h>
#include <ParticleSystemData3.h>

namespace CalfFluidEngine {
	class ParticleSystemSolver3 : public PhysicsAnimation{
	public:
		ParticleSystemSolver3();
		virtual ~ParticleSystemSolver3();
		ParticleSystemSolver3(double radius,double mass);
	private:
		void timeStepStart(double timeStepInSeconds);
		void timeStepEnd(double timeStepInSeconds);
		void timeIntegration(double timeIntervalInSeconds);
		void resolveCollision();
		void updateCollider(double timeStepInSeconds);
		ParticleSystemData3::VectorData _newPositions;
		ParticleSystemData3::VectorData _newVelocities;
	protected:
		void onTimeStep(double timeIntervalInSeconds) override;
		virtual void onInitialize() override;
		void setParticleSystemData(const std::shared_ptr<ParticleSystemData3>& newParticles);
		std::shared_ptr<ParticleSystemData3> GetParticleSystemData() const;
		//**********************************************
		//the function is called in ParticleSystemSolver3:timeStepStart(double);
		// Called when a time-step is about to begin;
		//**********************************************
		virtual void onTimeStepStart(double timeStepInSeconds);

		//**********************************************
		//the function is called in ParticleSystemSolver3:timeStepEnd(double);
		// Called when a time-step is about to end;
		//**********************************************
		virtual void onTimeStepEnd(double timeStepInSeconds);

		//**********************************************
		//the function is called in ParticleSystemSolver3:onTimeStep(double);
		//accumulate forces
		//**********************************************
		virtual void accumulateForces(double timeIntervalInSeconds);

		std::shared_ptr<ParticleSystemData3> _particleSystemData;
		std::shared_ptr<VectorField3> _wind;
        Vector3D _gravity = Vector3D(0.0, -9.8, 0.0);
		double _dragCoefficient = 1e-4;
		std::shared_ptr<Collider3> _collider;
		double _restitutionCoefficient = 0.0;
	};
}
#endif
