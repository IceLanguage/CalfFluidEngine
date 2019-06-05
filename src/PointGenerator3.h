#ifndef _CalfFluidEngine_PointGenerator3_
#define _CalfFluidEngine_PointGenerator3_
#include <BoundingBox3.h>
#include <vector>
#include <functional>
namespace CalfFluidEngine {
	class PointGenerator3
	{
	public:
		PointGenerator3();
		virtual ~PointGenerator3();

		void Generate(
			const BoundingBox3D& boundingBox,
			double spacing,
			std::vector<Vector3D>* points) const;

		//**********************************************
		// This function iterates every point within the bounding box and invokes
		// The input parameter of the callback function is the 
		// position of the point and the return value tells whether 
		// the iteration should stop or not.
		//**********************************************
		virtual void ForEachPoint(
			const BoundingBox3D& boundingBox,
			double spacing,
			const std::function<bool(const Vector3D&)>& callback) const = 0;
	};
}
#endif