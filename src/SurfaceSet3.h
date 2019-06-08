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

	private:
		virtual Vector3D closestPointLocal(const Vector3D& otherPoint) const;

		void buildBVH() const;
		std::vector<std::shared_ptr<Surface3>> _surfaces;
		mutable BVH3<std::shared_ptr<Surface3>> _bvh;
	};
}
#endif
