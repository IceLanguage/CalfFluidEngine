#ifndef _CalfFluidEngine_Collider3_
#define _CalfFluidEngine_Collider3_

#include <Vector3.h>
#include <memory>
#include <Surface3.h>

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
	private:
		std::shared_ptr<Surface3> _surface;
		double _frictionCoeffient = 0.0;
	};
}
#endif