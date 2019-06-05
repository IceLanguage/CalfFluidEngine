#include "VolumeParticleEmitter3.h"
#include <tbb\parallel_for.h>
#include <tbb\blocked_range.h>

using namespace CalfFluidEngine;

static const size_t kDefaultHashGridResolution = 64;

VolumeParticleEmitter3::VolumeParticleEmitter3()
{
}


VolumeParticleEmitter3::~VolumeParticleEmitter3()
{
}

void CalfFluidEngine::VolumeParticleEmitter3::onUpdate(double currentTimeInSeconds, double timeIntervalInSeconds)
{
	auto particles = GetTarget();

	if (particles == nullptr) {
		return;
	}

	if (_numberOfEmittedParticles > 0 && _isOneShot) {
		return;
	}

	std::vector<Vector3D> newPositions;
	std::vector<Vector3D> newVelocities;
	std::vector<Vector3D> Forces;
	emit(particles, &newPositions, &newVelocities);

	particles->AddParticles(newPositions, newVelocities, Forces);
}

void CalfFluidEngine::VolumeParticleEmitter3::emit(const std::shared_ptr<ParticleSystemData3>& particles, std::vector<Vector3D>* newPositions, std::vector<Vector3D>* newVelocities)
{
	if (_surface == nullptr) return;
	_surface->Update();

	BoundingBox3D region = _bounds;
	if (_surface != nullptr) {
		BoundingBox3D surfaceBox = _surface->GetBoundingBox();
		region.lowerCorner = max(region.lowerCorner, surfaceBox.lowerCorner);
		region.upperCorner = min(region.upperCorner, surfaceBox.upperCorner);
	}

	const double j = GetJitter();

	if (_allowOverlapping || _isOneShot) 
	{
	}
	else 
	{
		PointHashGridSearcher3 neighborSearcher(
			Vector3<size_t>(
				kDefaultHashGridResolution, 
				kDefaultHashGridResolution,
				kDefaultHashGridResolution),
			2.0 * _spacing);
		if (!_allowOverlapping) {
			neighborSearcher.Build(particles->GetPositions());
		}
	}

	newVelocities->resize(newPositions->size());

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, newVelocities->size()),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
			(*newVelocities)[i] = velocityAt((*newPositions)[i]);
	});
}

Vector3D CalfFluidEngine::VolumeParticleEmitter3::velocityAt(const Vector3D & point) const
{
	Vector3D r = point - _surface->transform.GetTranslation();
	return _linearVel + Vector3D::Cross(_angularVel,r) + _initialVel;
}
