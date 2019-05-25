#include "ParticleSystemSolver3.h"
#include <Constant.h>
#include <tbb\parallel_for.h>
#include <tbb\blocked_range.h>

using namespace CalfFluidEngine;

ParticleSystemSolver3::ParticleSystemSolver3() 
	:ParticleSystemSolver3(kEpsilonD, kEpsilonD)
{
}


ParticleSystemSolver3::~ParticleSystemSolver3()
{
}

ParticleSystemSolver3::ParticleSystemSolver3(double radius, double mass)
{
	_particleSystemData = std::make_shared<ParticleSystemData3>();
	_particleSystemData->SetParticleRadius(radius);
	_particleSystemData->SetParticleMass(mass);
	_wind = std::make_shared<ConstantVectorField3>(Vector3D());
}

void CalfFluidEngine::ParticleSystemSolver3::timeStepStart(double timeStepInSeconds)
{
	auto forces = _particleSystemData->GetForces();

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, forces.size()),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
			forces[i] = Vector3D::zero;
	});

	updateCollider(timeStepInSeconds);

	size_t n = _particleSystemData->GetNumberOfParticles();
	_newPositions.resize(n);
	_newVelocities.resize(n);

	onTimeStepStart(timeStepInSeconds);
}


void CalfFluidEngine::ParticleSystemSolver3::timeStepEnd(double timeStepInSeconds)
{
	size_t n = _particleSystemData->GetNumberOfParticles();
	auto positions = _particleSystemData->GetPositions();
	auto velocities = _particleSystemData->GetVelocities();

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, n),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
		{
			positions[i] = _newPositions[i];
			velocities[i] = _newVelocities[i];
		}
			
	});

	onTimeStepEnd(timeStepInSeconds);
}

void CalfFluidEngine::ParticleSystemSolver3::onTimeStepStart(double timeStepInSeconds)
{
}

void CalfFluidEngine::ParticleSystemSolver3::onTimeStepEnd(double timeStepInSeconds)
{
}

void CalfFluidEngine::ParticleSystemSolver3::accumulateForces(double timeIntervalInSeconds)
{
	size_t n = _particleSystemData->GetNumberOfParticles();
	auto forces = _particleSystemData->GetForces();
	auto velocities = _particleSystemData->GetVelocities();
	auto positions = _particleSystemData->GetPositions();
	const double mass = _particleSystemData->GetParticleMass();

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, n),
		[&](const tbb::blocked_range<size_t> & b) {
			for (size_t i = b.begin(); i != b.end(); ++i){
				// Gravity
				Vector3D force = mass * _gravity;

				// Wind forces
				Vector3D relativeVel = velocities[i] - _wind->Sample(positions[i]);
				force += -_dragCoefficient * relativeVel;

				forces[i] += force;
		}
		
	});
}

void CalfFluidEngine::ParticleSystemSolver3::timeIntegration(double timeIntervalInSeconds)
{
	size_t n = _particleSystemData->GetNumberOfParticles();
	auto forces = _particleSystemData->GetForces();
	auto velocities = _particleSystemData->GetVelocities();
	auto positions = _particleSystemData->GetPositions();
	const double mass = _particleSystemData->GetParticleMass();

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, n),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i) {
			Vector3D& newVelocity = _newVelocities[i];
			newVelocity = velocities[i];
			newVelocity.AddScaledVector(forces[i] / mass, timeIntervalInSeconds);

			Vector3D& newPosition = _newPositions[i];
			newPosition = positions[i];
			newPosition.AddScaledVector(newVelocity, timeIntervalInSeconds);
		}

	});
}

void CalfFluidEngine::ParticleSystemSolver3::resolveCollision()
{
	if (_collider != nullptr) {
		size_t numberOfParticles = _particleSystemData->GetNumberOfParticles();
		const double radius = _particleSystemData->GetParticleRadius();

		tbb::parallel_for(
			tbb::blocked_range<size_t>(size_t(0), numberOfParticles),
			[&](const tbb::blocked_range<size_t> & b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
			{
				_collider->ResolveCollision(
					radius,
					_restitutionCoefficient,
					&_newPositions[i],
					&_newVelocities[i]);
			}
				
		});
	}
}

void CalfFluidEngine::ParticleSystemSolver3::updateCollider(double timeStepInSeconds)
{

}


void CalfFluidEngine::ParticleSystemSolver3::onTimeStep(double timeIntervalInSeconds)
{
	timeStepStart(timeIntervalInSeconds);

	accumulateForces(timeIntervalInSeconds);
	timeIntegration(timeIntervalInSeconds);
	resolveCollision();

	timeStepEnd(timeIntervalInSeconds);
}

void CalfFluidEngine::ParticleSystemSolver3::onInitialize()
{
	updateCollider(0.0);
}

