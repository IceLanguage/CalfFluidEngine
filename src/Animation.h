#ifndef _CalfFluidEngine_Animation_
#define _CalfFluidEngine_Animation_
namespace CalfFluidEngine {

	struct Frame final {
		int index = 0;
		double timeIntervalInSeconds = 1.0 / 60.0;
		Frame();
		Frame(int newIndex, double newTimeIntervalInSeconds);
		double GetTimeInSeconds() const;
		void Advance();
		void Advance(int delta);
	};

	class Animation{
	public:
		Animation();
		virtual ~Animation();
		void Update(const Frame& frame);
	protected:

		//**********************************************
		//the function is called from Animation:Update();
		//this function and implement its logic for updating the animation state.
		//**********************************************
		virtual void OnUpdate(const Frame& frame) = 0;
	};
}
#endif