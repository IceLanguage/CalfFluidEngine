#ifndef _CalfFluidEngine_GridSystemData3_
#define _CalfFluidEngine_GridSystemData3_
#include <Grid3.h>
namespace CalfFluidEngine {
	class GridSystemData3
	{
	public:
		GridSystemData3();
		GridSystemData3(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing,
			const Vector3D& origin);
		virtual ~GridSystemData3();
		void Resize(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing,
			const Vector3D& origin);
		Vector3<size_t> GetResolution() const { return _resolution; }
		Vector3D GetGridSpacing() const { return _gridSpacing; }
		Vector3D GetOrigin() const { return _origin; }
		BoundingBox3D GetBoundingBox() const;
		const std::shared_ptr<FaceCenteredGrid3>& GetVelocity() const { return _velocity; }
		const std::shared_ptr<ScalarGrid3>& ScalarDataAt(size_t idx) const;
		const std::shared_ptr<VectorGrid3>& VectorDataAt(size_t idx) const;
		size_t GetNumberOfScalarData() const;
		size_t GetNnumberOfVectorData() const;
	private:
		Vector3<size_t> _resolution;
		Vector3D _gridSpacing;
		Vector3D _origin;
		size_t _velocityIdx;
		std::shared_ptr<FaceCenteredGrid3> _velocity;
		std::vector<std::shared_ptr<ScalarGrid3>> _scalarDataList;
		std::vector<std::shared_ptr<VectorGrid3>> _vectorDataList;
	};

}
#endif