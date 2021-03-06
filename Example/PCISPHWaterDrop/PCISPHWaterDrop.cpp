// PCISPHWaterDrop.cpp: 定义控制台应用程序的入口点。
//


#include <BoundingBox3.h>
#include <PCISPHSolver3.h>
#include <SurfaceSet3.h>
#include <Plane3.h>
#include <Sphere3.h>
#include <VolumeParticleEmitter3.h>
#include <OutputForMatplotlib.h>
#include <Box3.h>
using namespace CalfFluidEngine;

int main()
{
	const double targetSpacing = 0.02;

	BoundingBox3D domain(Vector3D::zero, Vector3D(1, 2, 0.5));

	PCISPHSolver3 solver;
	solver.SetPseudoViscosityCoefficient(0.0);

	std::shared_ptr<SphSystemData3> particles = solver.GetSphData();
	particles->SetTargetDensity(1000.0);
	particles->SetTargetSpacing(targetSpacing);

	std::shared_ptr<SurfaceSet3> surfaceSet = std::make_shared<SurfaceSet3>();
	surfaceSet->AddSurface(
		std::make_shared<Plane3>(
			Vector3D(0, 1, 0), Vector3D(0, 0.25 * domain.GetHeight(), 0)));
	surfaceSet->AddSurface(
		std::make_shared<Sphere3>(
			domain.GetMidPoint(), 0.15 * domain.GetWidth()));

	BoundingBox3D sourceBound(domain);
	sourceBound.Expand(-targetSpacing);


	auto emitter = std::make_shared<VolumeParticleEmitter3>();
	emitter->SetSurface(surfaceSet);
	emitter->SetBoundingBox3D(sourceBound);
	emitter->SetSpacing(targetSpacing);
	solver.SetEmitter(emitter);

	std::shared_ptr<Box3> box = std::make_shared<Box3>(domain);
	box->isNormalFlipped = true;
	std::shared_ptr<RigidBodyCollider3> collider = std::make_shared<RigidBodyCollider3>(box);
	solver.SetCollider(collider);

	OutputForMatplotlib o("PCISPHWaterDrop");
	o.SaveParticleDataXY(particles, 0);

	for (Frame frame(0, 1.0 / 60.0); frame.index < 100; frame.Advance()) {
		solver.Update(frame);

		o.SaveParticleDataXY(particles, frame.index);
	}
    return 0;
}

