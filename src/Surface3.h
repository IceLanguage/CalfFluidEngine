#ifndef _CalfFluidEngine_Surface3_
#define _CalfFluidEngine_Surface3_
#include <Vector3.h>
#include <Ray3.h>
#include <Constant.h>
#include <Transform3.h>
#include <BoundingBox3.h>
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
		Surface3(const Transform3& transform_, bool isNormalFlipped_ = false)
			: transform(transform_), isNormalFlipped(isNormalFlipped_) {}
		Surface3(const Surface3& other)
			: transform(other.transform), isNormalFlipped(other.isNormalFlipped) {}

		//**********************************************
		// Maybe updates internal spatial query engine.
		//**********************************************
		virtual void Update() {}

		Vector3D GetClosestPoint(const Vector3D& otherPoint) const;

		Vector3D GetClosestNormal(const Vector3D& otherPoint) const;

		double GetClosestDistance(const Vector3D& otherPoint) const;

		//**********************************************
		// Check if ray Intersects the surface
		//**********************************************
		virtual bool Intersects(const Ray3D& ray) const;

		virtual SurfaceRayIntersection3 GetClosestIntersection(const Ray3D& ray);

		bool IsInside(const Vector3D& otherPoint) const;

		BoundingBox3D GetBoundingBox() const;

		double SignedDistance(const Vector3D& otherPoint) const;
		
		virtual bool IsBounded() const { return true; }
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

		//**********************************************
		// Returns true if otherPoint in local is inside by given depth the volume
		//**********************************************
		virtual bool isInsideLocal(const Vector3D& otherPoint) const;

		//**********************************************
		// Returns the bounding box of this surface object in local frame.
		//**********************************************
		virtual BoundingBox3D getBoundingBoxLocal() const = 0;

		//**********************************************
		//Returns signed distance from the given point
		//**********************************************
		virtual double signedDistanceLocal(const Vector3D& otherPoint) const;
	};
}
#endif
