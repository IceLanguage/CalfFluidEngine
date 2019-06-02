// SphTest.cpp: 定义控制台应用程序的入口点。
//


#include <Plane3.h>
#include <SphSystemSolver3.h>
#include <BoundingBox3.h>
#include <memory>
#include <SphSystemData3.h>
#include <VolumeParticleEmitter3.h>
#include <OutputForMatplotlib.h>
#include <tbb\tbb.h>
using namespace CalfFluidEngine;
int main()
{
	SphSystemSolver3 solver;
	solver.SetViscosityCoefficient(0.1);
	solver.SetPseudoViscosityCoefficient(10);

	std::shared_ptr<SphSystemData3> particles = solver.GetSphData();
	const double targetSpacing = particles->GetTargetSpacing();

	BoundingBox3D initialBound(Vector3D::zero, Vector3D(1, 0.5, 1));
	initialBound.Expand(-targetSpacing);

	OutputForMatplotlib o("SphSystemSolver3.SphTest");
	o.SaveParticleDataXY(particles, 0);
	for (Frame frame(0, 1.0 / 60.0); frame.index < 100; frame.Advance()) {
		solver.Update(frame);

		o.SaveParticleDataXY(particles, frame.index);
	}
	return 0;
}

