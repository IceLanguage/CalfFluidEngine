// BounceParticle.cpp: 定义控制台应用程序的入口点。
//


#include <Plane3.h>
#include <memory>
#include <ParticleSystemSolver3.h>
#include <OutputForMatplotlib.h>
#include <Collider3.h>
using namespace CalfFluidEngine;

int main()
{
	OutputForMatplotlib o("BounceParticle"); 
	std::shared_ptr<Plane3> plane = std::make_shared<Plane3>(Vector3D(0, 1, 0), Vector3D::zero);
	std::shared_ptr<RigidBodyCollider3> collider
		= std::make_shared<RigidBodyCollider3>(plane);

	ParticleSystemSolver3 solver;
	solver.SetCollider(collider);
	solver.SetDragCoefficient(0.0);
	solver.SetRestitutionCoefficient(0.7);

	std::shared_ptr<ParticleSystemData3> particles = solver.GetParticleSystemData();
	particles->AddParticle({ 0.0, 3.0, 0.0 }, { 1.0, 0.0, 0.0 });

	std::vector<double> x(1000);
	std::vector<double> y(1000);
	char filename[256];
	

	Frame frame;
	frame.timeIntervalInSeconds = 1.0 / 300.0;
	x[frame.index] = particles->GetPositions()[0].x;
	y[frame.index] = particles->GetPositions()[0].y;
	snprintf(filename, sizeof(filename), "data.#line2,0000,x.npy");
	o.SaveData(x, 0, filename);
	snprintf(filename, sizeof(filename), "data.#line2,0000,y.npy");
	o.SaveData(y, 0, filename);
	frame.Advance();
	for (; frame.index < 1000; frame.Advance()) {
		solver.Update(frame);

		x[frame.index] = particles->GetPositions()[0].x;
		y[frame.index] = particles->GetPositions()[0].y;
		snprintf(
			filename,
			sizeof(filename),
			"data.#line2,%04d,x.npy",
			frame.index);
		o.SaveData(x, frame.index, filename);
		snprintf(
			filename,
			sizeof(filename),
			"data.#line2,%04d,y.npy",
			frame.index);
		o.SaveData(y, frame.index, filename);
	}
    return 0;
}

