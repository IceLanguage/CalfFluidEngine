#ifndef _CalfFluidEngine_Box3_
#define _CalfFluidEngine_Box3_
#include <Surface3.h>
namespace CalfFluidEngine {
	class Box3 :public Surface3
	{
	public:
		BoundingBox3D bound
			= BoundingBox3D(Vector3D::zero, Vector3D(1.0, 1.0, 1.0));
		Box3(
			const Transform3& transform = Transform3(),
			bool isNormalFlipped = false) 
			: Surface3(transform, isNormalFlipped) {}
		Box3(
			const Vector3D& lowerCorner,
			const Vector3D& upperCorner,
			const Transform3& transform = Transform3(),
			bool isNormalFlipped = false)
			: Box3(BoundingBox3D(lowerCorner, upperCorner), transform,
				isNormalFlipped) {}
		explicit Box3(
			const BoundingBox3D& boundingBox,
			const Transform3& transform = Transform3(),
			bool isNormalFlipped = false)
			: Surface3(transform, isNormalFlipped), bound(boundingBox) {}
		Box3(){}
		virtual ~Box3(){}
		virtual bool Intersects(const Ray3D& ray) const override;
	protected:
		virtual Vector3D closestPointLocal(const Vector3D& otherPoint) const;
		virtual Vector3D closestNormalLocal(const Vector3D& otherPoint) const;
		virtual SurfaceRayIntersection3 closestIntersectionLocal(
			const Ray3D& ray) const;
		virtual BoundingBox3D getBoundingBoxLocal() const;
	};
}
#endif
