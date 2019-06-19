#include "PCISPHSolver3.h"
#include "SphKernel.h"
#include "PointGenerator3.h"
using namespace CalfFluidEngine;

PCISPHSolver3::PCISPHSolver3()
{
	SetTimeStepLimitScale(5.0);
}


PCISPHSolver3::~PCISPHSolver3()
{
}

void CalfFluidEngine::PCISPHSolver3::accumulatePressureForce(double timeStepInSeconds)
{
	auto particles = GetSphData();
	const size_t numberOfParticles = particles->GetNumberOfParticles();
	const double delta = computeDelta(timeStepInSeconds);
	const double targetDensity = particles->GetDensity();
	const double mass = particles->GetParticleMass();

	auto& p = particles->GetPressures();
	auto& d = particles->GetDensities();
	auto& x = particles->GetPositions();
	auto& v = particles->GetVelocities();
	auto& f = particles->GetForces();

	//Initialize 
	std::vector<double> predictedDensities(numberOfParticles, 0.0);

	SphStandardKernel3 kernel(particles->GetKernelRadius());

	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
		{
			p[i] = 0.0;
			_pressureForces[i] = Vector3D::zero;
			_densityErrors[i] = 0.0;
			predictedDensities[i] = d[i];
		}
	});

	for (unsigned int k = 0; k < _maxNumberOfIterations; ++k) 
	{
		// Predict velocity and position
		tbb::parallel_for(
			tbb::blocked_range<size_t>(0, numberOfParticles),
			[&](const tbb::blocked_range<size_t> & b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
			{
				_tempVelocities[i] = v[i] + 
					(f[i] + _pressureForces[i]) / mass * timeStepInSeconds;
				_tempPositions[i] = x[i] + 
					 _tempVelocities[i] * timeStepInSeconds;
			}
		});

		// Resolve collisions
		ParticleSystemSolver3::resolveCollision(_tempPositions, _tempVelocities);

		// Compute pressure from density error
		tbb::parallel_for(
			tbb::blocked_range<size_t>(0, numberOfParticles),
			[&](const tbb::blocked_range<size_t> & b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
			{
				double weightSum = 0.0;
				const auto& neighbors = particles->GetNeighborLists()[i];

				for (size_t j : neighbors) {
					double dist = Vector3D::Distance(_tempPositions[j], _tempPositions[i]);
					weightSum += kernel(dist);
				}
	
				weightSum += kernel(0);

				double density = mass * weightSum;
				double densityError = (density - targetDensity);
				double pressure = delta * densityError;

				if (pressure < 0.0) {
					pressure *= _negativePressureScale;
					densityError *= _negativePressureScale;
				}

				p[i] += pressure;
				predictedDensities[i] = density;
				_densityErrors[i] = densityError;
			}
		});

		// Compute pressure gradient force 
		tbb::parallel_for(
			tbb::blocked_range<size_t>(0, numberOfParticles),
			[&](const tbb::blocked_range<size_t> & b) {
			for (size_t i = b.begin(); i != b.end(); ++i)
			{
				_pressureForces[i] = Vector3D::zero;
			}
		});
		
		SphSystemSolver3::accumulatePressureForce(x, predictedDensities, p, _pressureForces);

		// Compute max density error
		double maxDensityError = 0.0;
		for (size_t i = 0; i < numberOfParticles; ++i) {
			maxDensityError = AbsMax(maxDensityError, _densityErrors[i]);
		}
		double densityErrorRatio = maxDensityError / targetDensity;


		if (std::fabs(densityErrorRatio) < _maxDensityErrorRatio){
			break;
		}
	}

	//Accumlate pressure force
	tbb::parallel_for(
		tbb::blocked_range<size_t>(0, numberOfParticles),
		[&](const tbb::blocked_range<size_t> & b) {
		for (size_t i = b.begin(); i != b.end(); ++i)
		{
			f[i] += _pressureForces[i];
		}
	});
}

double CalfFluidEngine::PCISPHSolver3::computeDelta(double timeStepInSeconds)
{
	auto particles = GetSphData();
	const double kernelRadius = particles->GetKernelRadius();

	std::vector<Vector3D> points;
	BccLatticePointGenerator pointsGenerator;
	Vector3D origin = Vector3D::zero;
	BoundingBox3D sampleBound(origin, origin);
	sampleBound.Expand(1.5 * kernelRadius);

	pointsGenerator.Generate(sampleBound, particles->GetTargetSpacing(), &points);

	SphSpikyKernel3 kernel(kernelRadius);

	double denom = 0;
	Vector3D denom1 = Vector3D::zero;
	double denom2 = 0;

	for (size_t i = 0; i < points.size(); ++i) {
		const Vector3D& point = points[i];
		double distanceSquared = point.SquareMagnitude();

		if (distanceSquared < kernelRadius * kernelRadius) {
			double distance = std::sqrt(distanceSquared);
			Vector3D direction =
				(distance > 0.0) ? point / distance : Vector3D::zero;

			// grad(Wij)
			Vector3D gradWij = kernel.Gradient(distance, direction);
			denom1 += gradWij;
			denom2 += Vector3D::Dot(gradWij,gradWij);
		}
	}

	denom += -Vector3D::Dot(denom1,denom1) - denom2;

	return (std::fabs(denom) > 0.0) ?
		-1 / (computeBeta(timeStepInSeconds) * denom) : 0;
}

double CalfFluidEngine::PCISPHSolver3::computeBeta(double timeStepInSeconds)
{
	auto particles = GetSphData();
	double t = particles->GetParticleMass() * timeStepInSeconds
		/ particles->GetDensity();
	return 2.0 * t * t;
}
