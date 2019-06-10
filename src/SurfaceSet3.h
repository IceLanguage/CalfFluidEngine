#ifndef _CalfFluidEngine_SurfaceSet3_
#define _CalfFluidEngine_SurfaceSet3_
#include <Surface3.h>
#include <vector>
#include <memory>
#include <BVH.h>
namespace CalfFluidEngine {
	class SurfaceSet3 : public Surface3
	{
	public:
		SurfaceSet3();
		virtual ~SurfaceSet3();
		void AddSurface(const std::shared_ptr<Surface3>& surface);
		size_t GetNumberOfSurfaces() const {
			return _surfaces.size();
		}
		const std::shared_ptr<Surface3>& GetSurfaceAt(size_t i) const{
			return _surfaces[i];
		}
		virtual bool Intersects(const Ray3D& ray) const override;
		void Update() override;
		virtual bool IsBounded() const override;
	private:
		virtual Vector3D closestPointLocal(const Vector3D& otherPoint) const;
		virtual Vector3D closestNormalLocal(const Vector3D& otherPoint) const;
		virtual SurfaceRayIntersection3 closestIntersectionLocal(
			const Ray3D& ray) const;
		virtual BoundingBox3D getBoundingBoxLocal() const;
		virtual double closestDistanceLocal(const Vector3D& otherPointLocal) const override;
		virtual bool isInsideLocal(const Vector3D& otherPoint) const override;
		double signedDistanceLocal(const Vector3D& otherPoint) const override;
		void buildBVH() const;
		std::vector<std::shared_ptr<Surface3>> _surfaces;
		std::vector<std::shared_ptr<Surface3>> _unboundedSurfaces;
		mutable BVH3<std::shared_ptr<Surface3>> _bvh;
	};
}
#endif
