#ifndef _CalfFluidEngine_ParticleSystemSolver3_
#define _CalfFluidEngine_ParticleSystemSolver3_

#include <PhysicsAnimation.h>
#include <vector>
#include <Vector3.h>
#include <memory>
#include <Field3.h>

namespace CalfFluidEngine {

	class ParticleSystemData3
	{
	public:
		typedef std::vector<double> ScalarData;
		typedef std::vector<Vector3D> VectorData;

		ParticleSystemData3();
		virtual ~ParticleSystemData3();
		explicit ParticleSystemData3(size_t numberOfParticles);
		void Resize(size_t newNumberOfParticles);
		size_t GetNumberOfParticles() const;
		size_t AddVectorData(const Vector3D& initialVal = Vector3D::zero);
		size_t AddScalarData(double initialVal);
		std::vector<Vector3D> GetPositions() const;
		std::vector<Vector3D> GetPositions();
		std::vector<Vector3D> GetVelocities() const;
		std::vector<Vector3D> GetVelocities();
		std::vector<Vector3D> GetForces() const;
		std::vector<Vector3D> GetForces();
		void AddParticle(
			const Vector3D& newPosition,
			const Vector3D& newVelocity,
			const Vector3D& newForce);
		void AddParticles(
			const std::vector<Vector3D>& newPositions,
			const std::vector<Vector3D>& newVelocities,
			const std::vector<Vector3D>& newForces);
		std::vector<double> ScalarDataAt(size_t idx) const;
		std::vector<double> ScalarDataAt(size_t idx);
		std::vector<Vector3D> VectorDataAt(size_t idx) const;
		std::vector<Vector3D> VectorDataAt(size_t idx);

		double GetParticleRadius() const;
		void SetParticleRadius(double newRadius);
		double GetParticleMass() const;
		void SetParticleMass(double newMass);
	protected:
		size_t _positionIdx;
		size_t _velocityIdx;
		size_t _forceIdx;
		size_t _numberOfParticles = 0;
		double _radius = 1e-3;
		double _mass = 1e-3;

		std::vector<ScalarData> _scalarDataList;
		std::vector<VectorData> _vectorDataList;
	};

	class ParticleSystemSolver3 : public PhysicsAnimation{
	public:
		ParticleSystemSolver3();
		virtual ~ParticleSystemSolver3();
		ParticleSystemSolver3(double radius,double mass);
	private:
		void TimeStepStart(double timeStepInSeconds);
		void TimeStepEnd(double timeStepInSeconds);
		void TimeIntegration(double timeIntervalInSeconds);
		void ResolveCollision();

		ParticleSystemData3::VectorData _newPositions;
		ParticleSystemData3::VectorData _newVelocities;
	protected:
		void OnTimeStep(double timeIntervalInSeconds) override;
		virtual void OnInitialize() override;

		//**********************************************
		//the function is called in ParticleSystemSolver3:TimeStepStart(double);
		// Called when a time-step is about to begin;
		//**********************************************
		virtual void OnTimeStepStart(double timeStepInSeconds);

		//**********************************************
		//the function is called in ParticleSystemSolver3:TimeStepEnd(double);
		// Called when a time-step is about to end;
		//**********************************************
		virtual void OnTimeStepEnd(double timeStepInSeconds);

		//**********************************************
		//the function is called in ParticleSystemSolver3:OnTimeStep(double);
		//accumulate forces
		//**********************************************
		virtual void AccumulateForces(double timeIntervalInSeconds);

		std::shared_ptr<ParticleSystemData3> _particleSystemData;
		std::shared_ptr<VectorField3> _wind;
        Vector3D _gravity = Vector3D(0.0, -9.8, 0.0);
		double _dragCoefficient = 1e-4;
	};
}
#endif
