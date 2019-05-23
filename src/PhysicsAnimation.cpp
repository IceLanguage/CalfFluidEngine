#include "PhysicsAnimation.h"

using namespace CalfFluidEngine;

PhysicsAnimation::PhysicsAnimation(){
	_currentFrame.index = -1;
}


PhysicsAnimation::~PhysicsAnimation(){
}

bool CalfFluidEngine::PhysicsAnimation::IsUsingFixedTimeSteps() const{
	return _isUsingFixedTimeSteps;
}

void CalfFluidEngine::PhysicsAnimation::SetIsUsingFixedTimeSteps(bool isUsing){
	_isUsingFixedTimeSteps = isUsing;
}

void CalfFluidEngine::PhysicsAnimation::SetNumberOfFixedTimeSteps(unsigned int numberOfSteps){
	_numberOfFixedTimeSteps = numberOfSteps;
}

unsigned int CalfFluidEngine::PhysicsAnimation::GetNumberOfFixedTimeSteps() const{
	return _numberOfFixedTimeSteps;
}

void PhysicsAnimation::Initialize(){
	OnInitialize();
}

void PhysicsAnimation::TimeStep(double timeIntervalInSeconds){
	_currentTime = _currentFrame.GetTimeInSeconds();

	if (_isUsingFixedTimeSteps){
		const double actualTimeInterval = timeIntervalInSeconds /
			static_cast<double>(_numberOfFixedTimeSteps);

		for (unsigned int i = 0; i < _numberOfFixedTimeSteps; ++i) {

			OnTimeStep(actualTimeInterval);
			_currentTime += actualTimeInterval;
		}
	}
	else {
		//Using adaptive sub-timesteps

		double remainingTime = timeIntervalInSeconds;
		while (remainingTime > kEpsilonD) {
			unsigned int numSteps = GetNumberOfSubTimeSteps(remainingTime);
			double actualTimeInterval =
				remainingTime / static_cast<double>(numSteps);

			OnTimeStep(actualTimeInterval);

			remainingTime -= actualTimeInterval;
			_currentTime += actualTimeInterval;
		}
	}
}


//**********************************************
//physics-based animations are history dependent 
//the next state is defined by the previous state;
//This is why PhysicsAnimation class takes a progressive approach 
//for updating its state with the function named TimeStep;
//**********************************************
void PhysicsAnimation::OnUpdate(const Frame & frame){
	if (frame.index > _currentFrame.index) {
		if (_currentFrame.index < 0) {
			Initialize();
		}

		int numberOfFrames = frame.index - _currentFrame.index;

		for (int i = 0; i < numberOfFrames; ++i) {
			TimeStep(frame.timeIntervalInSeconds);
		}

		_currentFrame = frame;
	}
}

unsigned int PhysicsAnimation::GetNumberOfSubTimeSteps(double timeIntervalInSeconds) const{
	return _numberOfFixedTimeSteps;
}
