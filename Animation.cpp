#include "Animation.h"

using namespace CalfFluidEngine;

Animation::Animation()
{
}

Animation::~Animation()
{
}

void Animation::Update(const Frame & frame)
{
	OnUpdate(frame);
}

Frame::Frame()
{
}

Frame::Frame(int newIndex, double newTimeIntervalInSeconds) : 
	index(newIndex), 
	timeIntervalInSeconds(newTimeIntervalInSeconds)
{
}

double Frame::GetTimeInSeconds() const
{
	return index * timeIntervalInSeconds;
}

void Frame::Advance()
{
	++index;
}

void Frame::Advance(int delta)
{
	index += delta;
}
