#include "ParticleSystemData3.h"
#include <Constant.h>
#include <tbb\parallel_for.h>
#include <tbb\blocked_range.h>

using namespace CalfFluidEngine;


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

	_neighborSearcher = std::make_shared<PointHashGridSearcher3>(
		Vector3<size_t>(64, 64, 64),
		2.0 * _radius);
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

void CalfFluidEngine::ParticleSystemData3::BuildNeighborSearcher(double maxSearchRadius)
{
	_neighborSearcher = std::make_shared<PointHashGridSearcher3>(
		Vector3<size_t>(64, 64, 64),
		2.0 * maxSearchRadius);

	_neighborSearcher->Build(GetPositions());
}

void CalfFluidEngine::ParticleSystemData3::BuildNeighborLists(double maxSearchRadius)
{
	size_t numberOfParticles = GetNumberOfParticles();
	_neighborLists.resize(numberOfParticles);

	auto points = GetPositions();
	for (size_t i = 0; i < numberOfParticles; ++i) {
		Vector3D origin = points[i];
		_neighborLists[i].clear();

		_neighborSearcher->ForEachNearbyPoint(
			origin,
			maxSearchRadius,
			[&](size_t j, const Vector3D&) {
			if (i != j) {
				_neighborLists[i].push_back(j);
			}
		});
	}
}

const std::shared_ptr<PointNeighborSearcher3>& CalfFluidEngine::ParticleSystemData3::GetNeighborSearcher() const
{
	return _neighborSearcher;
}

const std::vector<std::vector<size_t>>& CalfFluidEngine::ParticleSystemData3::GetNeighborLists() const
{
	return _neighborLists;
}


