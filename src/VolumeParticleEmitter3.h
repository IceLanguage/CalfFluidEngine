#ifndef _CalfFluidEngine_VolumeParticleEmitter3_
#define _CalfFluidEngine_VolumeParticleEmitter3_
#include <ParticleEmitter3.h>
namespace CalfFluidEngine {
	class VolumeParticleEmitter3 final : public ParticleEmitter3
	{
	public:
		VolumeParticleEmitter3();
		virtual ~VolumeParticleEmitter3();
	private:
		void onUpdate(
			double currentTimeInSeconds,
			double timeIntervalInSeconds) override;
		void emit(
			const std::shared_ptr<ParticleSystemData3>& particles,
			std::vector<Vector3D>* newPositions,
			std::vector<Vector3D>* newVelocities);
		size_t _numberOfEmittedParticles = 0;
	};
}
#endif