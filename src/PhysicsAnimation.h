#ifndef _CalfFluidEngine_PhysicsAnimation_
#define _CalfFluidEngine_PhysicsAnimation_


#include <Animation.h>


namespace CalfFluidEngine{
	class PhysicsAnimation : public Animation{
	public:
		PhysicsAnimation();
		virtual ~PhysicsAnimation();
		bool IsUsingFixedTimeSteps() const;
		void SetIsUsingFixedTimeSteps(bool isUsing);
		void SetNumberOfFixedTimeSteps(unsigned int numberOfSteps);
		unsigned int GetNumberOfFixedTimeSteps() const;
	private:
		void initialize();

		void timeStep(double timeIntervalInSeconds);

		Frame _currentFrame;
		double _currentTime = 0.0;
		bool _isUsingFixedTimeSteps = true;
		unsigned int _numberOfFixedTimeSteps = 1;
	protected:
		virtual void onUpdate(const Frame& frame) override final;

		//**********************************************
		//the function is called from Animation:imeStep(double);
		//This function is called for each time-step;
		//**********************************************
		virtual void onTimeStep(double timeIntervalInSeconds) = 0;

		//**********************************************
		//the function is called from Animation:initialize();
		//Called at frame 0 to initialize the physics state.;
		//**********************************************
		virtual void onInitialize() = 0;

		//**********************************************
		//Returns the required number of sub-timesteps for given time
		//**********************************************
		virtual unsigned int getNumberOfSubTimeSteps(
			double timeIntervalInSeconds) const;
	};
}
#endif