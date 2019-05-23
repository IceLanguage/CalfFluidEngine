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


