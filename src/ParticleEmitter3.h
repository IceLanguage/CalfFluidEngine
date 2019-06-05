#ifndef _CalfFluidEngine_ParticleEmitter3_
#define _CalfFluidEngine_ParticleEmitter3_
#include <functional>
#include <ParticleSystemData3.h>
#include <memory>
namespace CalfFluidEngine {
	class ParticleEmitter3
	{
	public:
		ParticleEmitter3();
		virtual ~ParticleEmitter3();
		void Update(double currentTimeInSeconds, double timeIntervalInSeconds);
		void SetTarget(const std::shared_ptr<ParticleSystemData3>& particles);
		std::shared_ptr<ParticleSystemData3> GetTarget() const { return _particles; }
		
	protected:
		//**********************************************
		//the function is called from ParticleEmitter3:Update();
		//this function and implement its logic for updating the ParticleEmitter3 state.
		//**********************************************
		virtual void onUpdate(
			double currentTimeInSeconds,
			double timeIntervalInSeconds) = 0;

		//**********************************************
		//Called when ParticleEmitter3::setTarget is executed.
		//**********************************************
		virtual void onSetTarget(const std::shared_ptr<ParticleSystemData3>& particles) = 0;
	private:
		typedef std::function<void(ParticleEmitter3*, double, double)>
			OnBeginUpdateCallback;
		OnBeginUpdateCallback _onBeginUpdateCallback;
		std::shared_ptr<ParticleSystemData3> _particles;
	};
}
#endif
