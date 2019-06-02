// SphTest.cpp: 定义控制台应用程序的入口点。
//


#include <Plane3.h>
#include <SphSystemSolver3.h>
#include <BoundingBox3.h>
#include <memory>
#include <SphSystemData3.h>
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
	return 0;
}

