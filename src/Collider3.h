#ifndef _CalfFluidEngine_Collider3_
#define _CalfFluidEngine_Collider3_

#include <Vector3.h>
#include <memory>
#include <Surface3.h>
#include <functional>
namespace CalfFluidEngine {
	class Collider3
	{
	public:
		Collider3();
		virtual ~Collider3();
		void ResolveCollision(
			double radius,
			double restitutionCoefficient,
			Vector3D* position,
			Vector3D* velocity);
		void Update(
			double currentTimeInSeconds,
			double timeIntervalInSeconds);
		//**********************************************
		//Returns the velocity of the collider at given point.
		//**********************************************
		virtual Vector3D VelocityAt(const Vector3D& point) const = 0;
	protected:
		struct ColliderQueryResult final {
			double distance;
			Vector3D point;
			Vector3D normal;
			Vector3D velocity;
		};

		void getClosestPoint(
			const std::shared_ptr<Surface3>& surface,
			const Vector3D& queryPoint,
			ColliderQueryResult* result) const;

		//Returns true if given point is in the opposite side of the surface.
		bool isPenetrating(
			const ColliderQueryResult& colliderPoint,
			const Vector3D& position,
			double radius);
		std::shared_ptr<Surface3> _surface;
	private:
		typedef std::function<void(Collider3*, double, double)>
			OnBeginUpdateCallback;
		OnBeginUpdateCallback _onUpdateCallback;
		
		double _frictionCoeffient = 0.0;
	};

	class RigidBodyCollider3 : public Collider3
	{
	public:
		Vector3D linearVelocity;
		Vector3D angularVelocity;
		explicit RigidBodyCollider3(const std::shared_ptr<Surface3>& surface)
		{
			_surface = surface;
		}
		Vector3D VelocityAt(const Vector3D& point) const override
		{
			Vector3D r = point - _surface->transform.GetTranslation();
			return linearVelocity + Vector3D::Cross(angularVelocity, r);
		}
	};
}
#endif