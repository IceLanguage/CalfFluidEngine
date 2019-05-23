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

void CalfFluidEngine::ParticleSystemSolver3::TimeStepStart(double timeStepInSeconds)
{
	auto forces = _particleSystemData->GetForces();

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, forces.size()),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
			forces[i] = Vector3D::zero;
	});

	size_t n = _particleSystemData->GetNumberOfParticles();
	_newPositions.resize(n);
	_newVelocities.resize(n);

	OnTimeStepStart(timeStepInSeconds);
}


void CalfFluidEngine::ParticleSystemSolver3::TimeStepEnd(double timeStepInSeconds)
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

	OnTimeStepEnd(timeStepInSeconds);
}

void CalfFluidEngine::ParticleSystemSolver3::OnTimeStepStart(double timeStepInSeconds)
{
}

void CalfFluidEngine::ParticleSystemSolver3::OnTimeStepEnd(double timeStepInSeconds)
{
}

void CalfFluidEngine::ParticleSystemSolver3::AccumulateForces(double timeIntervalInSeconds)
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

void CalfFluidEngine::ParticleSystemSolver3::TimeIntegration(double timeIntervalInSeconds)
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

void CalfFluidEngine::ParticleSystemSolver3::ResolveCollision()
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


void CalfFluidEngine::ParticleSystemSolver3::OnTimeStep(double timeIntervalInSeconds)
{
	TimeStepStart(timeIntervalInSeconds);

	AccumulateForces(timeIntervalInSeconds);
	TimeIntegration(timeIntervalInSeconds);
	ResolveCollision();

	TimeStepEnd(timeIntervalInSeconds);
}

void CalfFluidEngine::ParticleSystemSolver3::OnInitialize()
{
}

CalfFluidEngine::ParticleSystemData3::ParticleSystemData3() 
	: ParticleSystemData3(0)
{
}

ParticleSystemData3::~ParticleSystemData3()
{
}

CalfFluidEngine::ParticleSystemData3::ParticleSystemData3(size_t numberOfParticles)
{
	_positionIdx = AddVectorData();
	_velocityIdx = AddVectorData();
	_forceIdx = AddVectorData();

	Resize(numberOfParticles);
}

void CalfFluidEngine::ParticleSystemData3::Resize(size_t newNumberOfParticles)
{
	_numberOfParticles = newNumberOfParticles;

	for (auto& attr : _scalarDataList) {
		attr.resize(newNumberOfParticles, 0.0);
	}

	for (auto& attr : _vectorDataList) {
		attr.resize(newNumberOfParticles, Vector3D::zero);
	}
}

size_t CalfFluidEngine::ParticleSystemData3::GetNumberOfParticles() const
{
	return _numberOfParticles;
}

size_t CalfFluidEngine::ParticleSystemData3::AddVectorData(const Vector3D & initialVal)
{
	size_t attrIdx = _vectorDataList.size();
	_vectorDataList.emplace_back(GetNumberOfParticles(), initialVal);
	return attrIdx;
}

size_t CalfFluidEngine::ParticleSystemData3::AddScalarData(double initialVal)
{
	size_t attrIdx = _scalarDataList.size();
	_scalarDataList.emplace_back(GetNumberOfParticles(), initialVal);
	return attrIdx;
}

std::vector<Vector3D> CalfFluidEngine::ParticleSystemData3::GetPositions() const
{
	return VectorDataAt(_positionIdx);
}

std::vector<Vector3D> CalfFluidEngine::ParticleSystemData3::GetPositions()
{
	return VectorDataAt(_positionIdx);
}

std::vector<Vector3D> CalfFluidEngine::ParticleSystemData3::GetVelocities() const
{
	return VectorDataAt(_velocityIdx);
}

std::vector<Vector3D> CalfFluidEngine::ParticleSystemData3::GetVelocities()
{
	return VectorDataAt(_velocityIdx);
}

std::vector<Vector3D> CalfFluidEngine::ParticleSystemData3::GetForces() const
{
	return VectorDataAt(_forceIdx);
}

std::vector<Vector3D> CalfFluidEngine::ParticleSystemData3::GetForces()
{
	return VectorDataAt(_forceIdx);
}

void CalfFluidEngine::ParticleSystemData3::AddParticle(const Vector3D & newPosition, const Vector3D & newVelocity, const Vector3D & newForce)
{
	std::vector<Vector3D> newPositions = { newPosition };
	std::vector<Vector3D> newVelocities = { newVelocity };
	std::vector<Vector3D> newForces = { newForce };

	AddParticles(newPositions, newVelocities, newForces);
}

void CalfFluidEngine::ParticleSystemData3::AddParticles(const std::vector<Vector3D>& newPositions, const std::vector<Vector3D>& newVelocities, const std::vector<Vector3D>& newForces)
{
	size_t oldNumberOfParticles = GetNumberOfParticles();
	size_t newNumberOfParticles = oldNumberOfParticles + newPositions.size();

	Resize(newNumberOfParticles);

	auto pos = GetPositions();
	auto vel = GetVelocities();
	auto frc = GetForces();

	tbb::parallel_for(tbb::blocked_range<size_t>(size_t(0),newPositions.size()),
		[&](const tbb::blocked_range<size_t> & b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
				pos[i + oldNumberOfParticles] = newPositions[i];
	});

	if (newVelocities.size() > 0) {
		tbb::parallel_for(tbb::blocked_range<size_t>(size_t(0), newPositions.size()),
			[&](const tbb::blocked_range<size_t> & b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
				vel[i + oldNumberOfParticles] = newVelocities[i];
		});
	}

	if (newForces.size() > 0) {
		tbb::parallel_for(tbb::blocked_range<size_t>(size_t(0), newPositions.size()),
			[&](const tbb::blocked_range<size_t> & b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
				frc[i + oldNumberOfParticles] = newForces[i];
		});
	}
}

std::vector<double> CalfFluidEngine::ParticleSystemData3::ScalarDataAt(size_t idx) const
{
	return _scalarDataList[idx];
}

std::vector<double> CalfFluidEngine::ParticleSystemData3::ScalarDataAt(size_t idx)
{
	return _scalarDataList[idx];
}

std::vector<Vector3D> CalfFluidEngine::ParticleSystemData3::VectorDataAt(size_t idx) const
{
	return _vectorDataList[idx];
}

std::vector<Vector3D> CalfFluidEngine::ParticleSystemData3::VectorDataAt(size_t idx)
{
	return _vectorDataList[idx];
}

double CalfFluidEngine::ParticleSystemData3::GetParticleRadius() const
{
	return _radius;
}

void CalfFluidEngine::ParticleSystemData3::SetParticleRadius(double newRadius)
{
	_radius = std::max(newRadius, 0.0);
}

double CalfFluidEngine::ParticleSystemData3::GetParticleMass() const
{
	return _mass;
}

void CalfFluidEngine::ParticleSystemData3::SetParticleMass(double newMass)
{
	_mass = std::max(newMass, 0.0);
}


