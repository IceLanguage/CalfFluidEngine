#ifndef _CalfFluidEngine_Surface3_
#define _CalfFluidEngine_Surface3_
#include <Vector3.h>
#include <Ray3.h>
#include <Constant.h>
#include <Transform3.h>
namespace CalfFluidEngine {
	struct SurfaceRayIntersection3 {
		bool isIntersecting = false;
		double distance = kMaxD;
		Vector3D point;
		Vector3D normal;
	};

	class Surface3
	{
	public:
		Transform3 transform;

		//! Flips normal when calling Surface3::GetClosestNormal(Vector3D).
		bool isNormalFlipped = false;
		Surface3();
		virtual ~Surface3();

		Vector3D GetClosestPoint(const Vector3D& otherPoint) const;

		Vector3D GetClosestNormal(const Vector3D& otherPoint) const;

		double GetClosestDistance(const Vector3D& otherPoint) const;

		bool Intersects(const Ray3D& ray) const;

		virtual SurfaceRayIntersection3 GetClosestIntersection(const Ray3D& ray) const = 0;
	protected:
		//**********************************************
		// Returns the closest point from the given point
		//**********************************************
		virtual Vector3D closestPointLocal(const Vector3D& otherPoint) const = 0;

		//**********************************************
		//Returns the normal to the closest point on the surface from the given
		//**********************************************
		virtual Vector3D closestNormalLocal(const Vector3D& otherPoint) const = 0;

		//**********************************************
		// Returns the closest intersection point for given ray in local 
		//**********************************************
		virtual SurfaceRayIntersection3 closestIntersectionLocal(
			const Ray3D& ray) const = 0;

		//**********************************************
		// Returns the closest distance from the given point in local
		//**********************************************
		virtual double closestDistanceLocal(const Vector3D& otherPointLocal) const;
	};
}
#endif
