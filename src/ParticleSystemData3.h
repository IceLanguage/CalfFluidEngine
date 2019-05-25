#ifndef _CalfFluidEngine_ParticleSystemData3_
#define _CalfFluidEngine_ParticleSystemData3_

#include <PhysicsAnimation.h>
#include <vector>
#include <Vector3.h>
#include <memory>
#include <Field3.h>
#include <Collider3.h>
#include <PointNeighborSearcher3.h>

namespace CalfFluidEngine {

	class ParticleSystemData3
	{
	public:
		typedef std::vector<double> ScalarData;
		typedef std::vector<Vector3D> VectorData;

		ParticleSystemData3();
		virtual ~ParticleSystemData3();
		explicit ParticleSystemData3(size_t numberOfParticles);
		void Resize(size_t newNumberOfParticles);
		size_t GetNumberOfParticles() const;
		size_t AddVectorData(const Vector3D& initialVal = Vector3D::zero);
		size_t AddScalarData(double initialVal = 0.0);
		std::vector<Vector3D> GetPositions() const;
		std::vector<Vector3D> GetPositions();
		std::vector<Vector3D> GetVelocities() const;
		std::vector<Vector3D> GetVelocities();
		std::vector<Vector3D> GetForces() const;
		std::vector<Vector3D> GetForces();
		void AddParticle(
			const Vector3D& newPosition,
			const Vector3D& newVelocity,
			const Vector3D& newForce);
		void AddParticles(
			const std::vector<Vector3D>& newPositions,
			const std::vector<Vector3D>& newVelocities,
			const std::vector<Vector3D>& newForces);
		std::vector<double> ScalarDataAt(size_t idx) const;
		std::vector<double> ScalarDataAt(size_t idx);
		std::vector<Vector3D> VectorDataAt(size_t idx) const;
		std::vector<Vector3D> VectorDataAt(size_t idx);

		double GetParticleRadius() const;
		void SetParticleRadius(double newRadius);
		double GetParticleMass() const;
		void SetParticleMass(double newMass);

		void BuildNeighborSearcher(double maxSearchRadius);
		void BuildNeighborLists(double maxSearchRadius);

		const std::shared_ptr<PointNeighborSearcher3> & GetNeighborSearcher() const;
	protected:
		size_t _positionIdx;
		size_t _velocityIdx;
		size_t _forceIdx;
		size_t _numberOfParticles = 0;
		double _radius = 1e-3;
		double _mass = 1e-3;

		std::vector<ScalarData> _scalarDataList;
		std::vector<VectorData> _vectorDataList;
		std::shared_ptr<PointNeighborSearcher3> _neighborSearcher;
		std::vector<std::vector<size_t>>_neighborLists;
 	};
}
#endif
