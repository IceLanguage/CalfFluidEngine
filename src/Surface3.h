#ifndef _CalfFluidEngine_Surface3_
#define _CalfFluidEngine_Surface3_
#include <Vector3.h>
#include <Ray3.h>
#include <Constant.h>
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
		Surface3();
		virtual ~Surface3();

		//**********************************************
		//Returns the closest point from the given Point 
		//**********************************************
		virtual Vector3D GetClosestPoint(const Vector3D& otherPoint) const;

		//**********************************************
		//Returns the normal to the closest point on the surface from the given
		//**********************************************
		virtual Vector3D GetClosestNormal(const Vector3D& otherPoint) const;

		//**********************************************
		//Returns the closest distance from the given point
		//**********************************************
		virtual double GetClosestDistance(const Vector3D& otherPoint) const;

		//**********************************************
		//Returns true if the given ray intersects with this surface object.
		//**********************************************
		virtual bool Intersects(const Ray3D& ray) const;

		virtual SurfaceRayIntersection3 GetClosestIntersection(const Ray3D& ray) const = 0;
	};
}
#endif
