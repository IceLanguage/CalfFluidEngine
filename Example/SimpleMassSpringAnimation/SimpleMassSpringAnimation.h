#ifndef _SimpleMassSpringAnimation_
#define _SimpleMassSpringAnimation_

#include <PhysicsAnimation.h>
#include <vector>
#include <Vector3.h>
#include <memory>
#include <Field3.h>

using namespace CalfFluidEngine;

class SimpleMassSpringAnimation : public PhysicsAnimation
{
public:
	struct Edge
	{
		size_t first;
		size_t second;
	};

	struct Constraint
	{
		size_t pointIndex;
		Vector3D fixedPosition;
		Vector3D fixedVelocity;
	};

	void Init(size_t numberOfPoints);
	void ExportStates(std::vector<double>& x, std::vector<double>& y) const
	{
		x.resize(positions.size());
		y.resize(positions.size());

		for (size_t i = 0; i < positions.size(); ++i)
		{
			x[i] = positions[i].x;
			y[i] = positions[i].y;
		}
	}

	std::vector<Vector3D> positions;
	std::vector<Vector3D> velocities;
	std::vector<Vector3D> forces;
	std::vector<Edge> edges;
	std::vector<Constraint> constraints;

	double mass = 1.0;
	Vector3D gravity = Vector3D(0.0, -9.8, 0.0);
	double stiffness = 500.0;
	double restLength = 1.0;
	double dampingCoefficient = 1.0;
	double dragCoefficient = 0.1;
	double floorPositionY = -7.0;
	double restitutionCoefficient = 0.3;

	std::shared_ptr<VectorField3> wind;

protected:
	void onTimeStep(double timeIntervalInSeconds) override;
	virtual void onInitialize() override {}
};

#endif