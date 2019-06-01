#include "ParticleEmitter3.h"

using namespace CalfFluidEngine;

ParticleEmitter3::ParticleEmitter3()
{
}


ParticleEmitter3::~ParticleEmitter3()
{
}

void ParticleEmitter3::Update(double currentTimeInSeconds, double timeIntervalInSeconds)
{
	if (_onBeginUpdateCallback) {
		_onBeginUpdateCallback(
			this, currentTimeInSeconds, timeIntervalInSeconds);
	}

	onUpdate(currentTimeInSeconds, timeIntervalInSeconds);
}

void CalfFluidEngine::ParticleEmitter3::SetTarget(const std::shared_ptr<ParticleSystemData3>& particles)
{
	_particles = particles;
	onSetTarget(particles);
}
