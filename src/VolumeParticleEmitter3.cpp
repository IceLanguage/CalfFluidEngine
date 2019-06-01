#include "VolumeParticleEmitter3.h"

using namespace CalfFluidEngine;

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

	if (_numberOfEmittedParticles > 0) {
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
}
