#include <algorithm>
#include <Animation.h>

using namespace CalfFluidEngine;

class SineAnimation : public Animation {
public:
	double x = 0.0;

protected:
	void OnUpdate(const Frame& frame) override {
		x = std::sin(10.0 * frame.GetTimeInSeconds());
	}
};

int main()
{
	SineAnimation sineAnim;
	for (Frame frame; frame.index < 240; frame.Advance()) {
		sineAnim.Update(frame);
	}
    return 0;
}

