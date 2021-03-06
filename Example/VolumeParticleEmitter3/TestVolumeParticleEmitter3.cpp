
#include <ParticleSystemSolver3.h>
#include <VolumeParticleEmitter3.h>
#include <OutputForMatplotlib.h>
#include <Sphere3.h>

using namespace CalfFluidEngine;

int main()
{
	OutputForMatplotlib o("TestVolumeParticleEmitter3");
	ParticleSystemSolver3 solver;

	std::shared_ptr<ParticleSystemData3> particles = solver.GetParticleSystemData();
	auto emitter = std::make_shared<VolumeParticleEmitter3>();
	emitter->SetSurface(std::make_shared<Sphere3>(Vector3D::zero, 1.0));
	emitter->SetBoundingBox3D(BoundingBox3D(Vector3D(-1, -1, -1), Vector3D(1, 1, 1)));
	emitter->SetSpacing(0.2);
	emitter->SetIsOneShot(false);
	emitter->SetIsAllowOverlapping(false);
	solver.SetEmitter(emitter);

	o.SaveParticleDataXY(particles, 0);

	for (Frame frame(0, 1.0 / 60.0); frame.index < 120; frame.Advance()) {
		solver.Update(frame);

		o.SaveParticleDataXY(particles, frame.index);
	}
    return 0;
}

