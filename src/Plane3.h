#ifndef _CalfFluidEngine_Plane3_
#define _CalfFluidEngine_Plane3_
#include <Surface3.h>
namespace CalfFluidEngine {
	class Plane3: public Surface3
	{
	public:
		Plane3();
		virtual ~ Plane3();
		Plane3(
			const Transform3& transform_, 
			bool isNormalFlipped_)
			: Surface3(transform_, isNormalFlipped_) {}
		Plane3(
			const Vector3D& normal, 
			const Vector3D& point,
			const Transform3& transform_ = Transform3(),
			bool isNormalFlipped_ = false)
			: Surface3(transform_, isNormalFlipped_), 
			_normal(normal),
			_NormalDotPoint(Vector3D::Dot(normal,point)) {}
		Plane3(
			const Vector3D& point0, 
			const Vector3D& point1,
			const Vector3D& point2, 
			const Transform3& transform_,
			bool isNormalFlipped_)
			: Surface3(transform_, isNormalFlipped_) {
			_normal = Vector3D::Cross((point1 - point0),(point2 - point0));
			_normal.Normalize();
			_NormalDotPoint = Vector3D::Dot(_normal, point0);
		}
		virtual bool Intersects(const Ray3D& ray) const override;
	protected:
		virtual Vector3D closestPointLocal(const Vector3D& otherPoint) const;
		virtual Vector3D closestNormalLocal(const Vector3D& otherPoint) const;
		virtual SurfaceRayIntersection3 closestIntersectionLocal(
			const Ray3D& ray) const;
		virtual BoundingBox3D getBoundingBoxLocal() const;
		Vector3D _normal = Vector3D(0, 1, 0);
		double _NormalDotPoint;
	};
}
#endif