#include "VolumeParticleEmitter3.h"
#include <tbb\parallel_for.h>
#include <tbb\blocked_range.h>

using namespace CalfFluidEngine;

static const size_t kDefaultHashGridResolution = 64;

// Returns randomly a point on a sphere.
template <typename T>
inline Vector3<T> uniformSampleSphere(T u1, T u2) {
	T y = 1 - 2 * u1;
	T r = std::sqrt(std::max<T>(0, 1 - y * y));
	T phi = static_cast<T>(2 * kPiD) * u2;
	T x = r * std::cos(phi);
	T z = r * std::sin(phi);
	return Vector3<T>(x, y, z);
}

double random(std::mt19937 rng) {
	std::uniform_real_distribution<> d(0.0, 1.0);
	return d(rng);
}

VolumeParticleEmitter3::VolumeParticleEmitter3()
{
	_pointGenerator = std::make_shared<BccLatticePointGenerator>();
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
	const double maxJitterDist = 0.5 * j * _spacing;

	if (_allowOverlapping || _isOneShot) 
	{
		_pointGenerator->ForEachPoint(region, _spacing, [&](const Vector3D& point) {
			Vector3D randomDir = uniformSampleSphere(random(_rng), random(_rng));
			Vector3D offset = maxJitterDist * randomDir;
			Vector3D candidate = point + offset;
			if (_surface->SignedDistance(candidate) <= 0.0) {
				if (_numberOfEmittedParticles < _maxNumberOfParticles) {
					newPositions->push_back(candidate);
					++_numberOfEmittedParticles;
				}
				else {
					return false;
				}
			}

			return true;
		});
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

		_pointGenerator->ForEachPoint(region, _spacing, [&](const Vector3D& point) {
			Vector3D randomDir = uniformSampleSphere(random(_rng), random(_rng));
			Vector3D offset = maxJitterDist * randomDir;
			Vector3D candidate = point + offset;
			if (_surface->SignedDistance(candidate) <= 0.0 &&
				(!_allowOverlapping &&!neighborSearcher.HasNearbyPoint(candidate, _spacing))) {
				if (_numberOfEmittedParticles < _maxNumberOfParticles) {
					newPositions->push_back(candidate);
					neighborSearcher.Add(candidate);
					++_numberOfEmittedParticles;
				}
				else {
					return false;
				}
			}

			return true;
		});
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
