#ifndef _CalfFluidEngine_Surface3_
#define _CalfFluidEngine_Surface3_
#include <Vector3.h>
namespace CalfFluidEngine {
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
		Vector3D GetClosestNormal(const Vector3D& otherPoint) const;

		//**********************************************
		//Returns the closest distance from the given point
		//**********************************************
		double GetClosestDistance(const Vector3D& otherPoint) const;
	};
}
#endif
