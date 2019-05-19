#include <algorithm>
#include <Animation.h>
#include <vector>
#include <OutputForMatplotlib.h>
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
	std::vector<double> t(240);
	std::vector<double> data(240);
	char filename[256];
	SineAnimation sineAnim;
	OutputForMatplotlib o("SineAnimation");
	for (Frame frame; frame.index < 4; frame.Advance()) {
		snprintf(
			filename,
			sizeof(filename),
			"data.#line2,%04d,x.npy",
			frame.index);
		o.SaveData(t, frame.index, filename);
		snprintf(
			filename,
			sizeof(filename),
			"data.#line2,%04d,y.npy",
			frame.index);
		o.SaveData(data, frame.index, filename);
		sineAnim.Update(frame);
	}
    return 0;
}

