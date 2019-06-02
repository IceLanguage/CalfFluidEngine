#ifndef _CalfFluidEngine_Sphere3_
#define _CalfFluidEngine_Sphere3_
#include <Surface3.h>
namespace CalfFluidEngine {
	class Sphere3 : public Surface3
	{
	public:
		Sphere3();
		Sphere3(
			const Transform3& transform_, 
			bool isNormalFlipped_)
			: Surface3(transform_, isNormalFlipped_) {}
		Sphere3(
			const Vector3D& center_, 
			double radius_,
			const Transform3& transform_, 
			bool isNormalFlipped_)
			: Surface3(transform_, isNormalFlipped_),
			center(center_),
			radius(radius_) {}
		virtual ~Sphere3();

		Vector3D center;
		double radius = 1.0;
	protected:
		virtual Vector3D closestPointLocal(const Vector3D& otherPoint) const;
		virtual Vector3D closestNormalLocal(const Vector3D& otherPoint) const;
		virtual SurfaceRayIntersection3 closestIntersectionLocal(
			const Ray3D& ray) const;
		virtual double closestDistanceLocal(const Vector3D& otherPointLocal) const override;
	};

}
#endif