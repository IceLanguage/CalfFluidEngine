#ifndef _CalfFluidEngine_VolumeParticleEmitter3_
#define _CalfFluidEngine_VolumeParticleEmitter3_
#include <ParticleEmitter3.h>
#include <BoundingBox3.h>
namespace CalfFluidEngine {
	class VolumeParticleEmitter3 final : public ParticleEmitter3
	{
	public:
		VolumeParticleEmitter3();
		virtual ~VolumeParticleEmitter3();
		bool IsOneShot() const { return _isOneShot; }
		void SetIsOneShot(bool newValue) { _isOneShot = newValue; }
		bool IsAllowOverlapping() const { return _allowOverlapping; }
		void SetIsAllowOverlapping(bool newValue) { _allowOverlapping = newValue; }
		void SetSurface(const std::shared_ptr<Surface3>& newSurface){ _surface = newSurface; }
		double GetJitter() const { return _jitter; }
		void SetJitter(double newJitter){ _jitter = Clamp(newJitter, 0.0, 1.0); }
		double GetSpacing() const { return _spacing; }
		void setSpacing(double newSpacing) { _spacing = newSpacing; }
	private:
		void onUpdate(
			double currentTimeInSeconds,
			double timeIntervalInSeconds) override;
		void emit(
			const std::shared_ptr<ParticleSystemData3>& particles,
			std::vector<Vector3D>* newPositions,
			std::vector<Vector3D>* newVelocities);
		Vector3D velocityAt(const Vector3D& point) const;

		size_t _numberOfEmittedParticles = 0;
		bool _isOneShot = true;
		bool _allowOverlapping = false;
		std::shared_ptr<Surface3> _surface;
		BoundingBox3D _bounds;
		double _jitter = 0.0;
		double _spacing;
		Vector3D _initialVel;
		Vector3D _linearVel;
		Vector3D _angularVel;
	};
}
#endif