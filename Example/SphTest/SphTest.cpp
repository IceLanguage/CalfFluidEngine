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
#include <Sphere3.h>
#include <Box3.h>
using namespace CalfFluidEngine;
int main()
{
	SphSystemSolver3 solver;
	solver.SetViscosityCoefficient(0.1);
	solver.SetPseudoViscosityCoefficient(10.0);

	std::shared_ptr<SphSystemData3> particles = solver.GetSphData();
	const double targetSpacing = particles->GetTargetSpacing();

	BoundingBox3D initialBound(Vector3D::zero, Vector3D(1, 0.5, 1));
	initialBound.Expand(-targetSpacing);

	auto emitter = std::make_shared<VolumeParticleEmitter3>();
	emitter->SetSurface(std::make_shared<Sphere3>(Vector3D::zero, 10.0));
	emitter->SetBoundingBox3D(initialBound);
	emitter->SetSpacing(targetSpacing);
	emitter->SetJitter(0.0);
	solver.SetEmitter(emitter);

	std::shared_ptr<Box3> box = std::make_shared<Box3>(Vector3D::zero, Vector3D(1, 1, 1));
	box->isNormalFlipped = true;
	std::shared_ptr<RigidBodyCollider3> collider = std::make_shared<RigidBodyCollider3>(box);

	solver.SetCollider(collider);

	OutputForMatplotlib o("SphTest");
	o.SaveParticleDataXY(particles, 0);
	for (Frame frame(0, 1.0 / 60.0); frame.index < 100; frame.Advance()) {
		solver.Update(frame);

		o.SaveParticleDataXY(particles, frame.index);
	}
	return 0;
}

