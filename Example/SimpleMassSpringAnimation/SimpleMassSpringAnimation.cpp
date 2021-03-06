// SimpleMassSpringAnimation.cpp: 定义控制台应用程序的入口点。
//

#include "SimpleMassSpringAnimation.h"
#include <OutputForMatplotlib.h>
#include <Constant.h>

int main()
{
	std::vector<double> xs, ys; 
	OutputForMatplotlib o("SimpleMassSpringAnimation");
	SimpleMassSpringAnimation anim;
	anim.Init(10);
	anim.wind = std::make_shared<ConstantVectorField3>(Vector3D(130.0, 0.0, 0.0));
	anim.constraints.push_back(SimpleMassSpringAnimation::Constraint{ 0, Vector3D(), Vector3D() });
	anim.ExportStates(xs, ys);

	char filename[256];
	snprintf(filename, sizeof(filename), "data.#line2,0000,x.npy");
	o.SaveData(xs, filename);
	snprintf(filename, sizeof(filename), "data.#line2,0000,y.npy");
	o.SaveData(ys, filename);

	for (Frame frame(0, 1.0 / 60.0); frame.index < 1080; frame.Advance())
	{
		anim.Update(frame);
		anim.ExportStates(xs, ys);

		snprintf(filename, sizeof(filename), "data.#line2,%04d,x.npy", frame.index);
		o.SaveData(xs, filename);
		snprintf(filename, sizeof(filename), "data.#line2,%04d,y.npy", frame.index);
		o.SaveData(ys, filename);
	}
    return 0;
}

void SimpleMassSpringAnimation::Init(size_t numberOfPoints)
{
	if (numberOfPoints == 0)
	{
		return;
	}

	size_t numberOfEdges = numberOfPoints - 1;

	positions.resize(numberOfPoints);
	velocities.resize(numberOfPoints);
	forces.resize(numberOfPoints);
	edges.resize(numberOfEdges);

	for (size_t i = 0; i < numberOfPoints; ++i)
	{
		positions[i].x = -static_cast<double>(i);
	}

	for (size_t i = 0; i < numberOfEdges; ++i)
	{
		edges[i] = Edge{ i, i + 1 };
	}
}

void SimpleMassSpringAnimation::onTimeStep(double timeIntervalInSeconds)
{
	size_t numberOfPoints = positions.size();
	size_t numberOfEdges = edges.size();

	// Compute forces
	for (size_t i = 0; i < numberOfPoints; ++i)
	{
		// Gravity force
		forces[i] = mass * gravity;

		// Air drag force
		Vector3D relativeVel = velocities[i];

		if (wind != nullptr)
		{
			relativeVel -= wind->Sample(positions[i]);
		}

		forces[i] += -dragCoefficient * relativeVel;
	}

	for (size_t i = 0; i < numberOfEdges; ++i)
	{
		size_t pointIndex0 = edges[i].first;
		size_t pointIndex1 = edges[i].second;

		// Compute spring force
		Vector3D pos0 = positions[pointIndex0];
		Vector3D pos1 = positions[pointIndex1];
		Vector3D r = pos0 - pos1;
		double distance = r.Magnitude();
		if (distance > kEpsilonD)
		{
			Vector3D force = -stiffness * (distance - restLength) * Vector3D::Normalize(r);
			forces[pointIndex0] += force;
			forces[pointIndex1] -= force;
		}

		// Add damping force
		Vector3D vel0 = velocities[pointIndex0];
		Vector3D vel1 = velocities[pointIndex1];
		Vector3D relativeVel0 = vel0 - vel1;
		Vector3D damping = -dampingCoefficient * relativeVel0;
		forces[pointIndex0] += damping;
		forces[pointIndex1] -= damping;
	}

	// Update states
	for (size_t i = 0; i < numberOfPoints; ++i)
	{
		// Compute new states
		Vector3D newAcceleration = forces[i] / mass;
		Vector3D newVelocity = velocities[i];
		newVelocity.AddScaledVector(newAcceleration, timeIntervalInSeconds);
		Vector3D newPosition = positions[i];
		newPosition.AddScaledVector(newVelocity, timeIntervalInSeconds);

		// Collision
		if (newPosition.y < floorPositionY)
		{
			newPosition.y = floorPositionY;

			if (newVelocity.y < 0.0)
			{
				newVelocity.y *= -restitutionCoefficient;
				newPosition.y += timeIntervalInSeconds * newVelocity.y;
			}
		}

		// Update states
		velocities[i] = newVelocity;
		positions[i] = newPosition;
	}

	for (size_t i = 0; i < constraints.size(); ++i)
	{
		size_t pointIndex = constraints[i].pointIndex;
		positions[pointIndex] = constraints[i].fixedPosition;
		velocities[pointIndex] = constraints[i].fixedVelocity;
	}
}
