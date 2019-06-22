#include "GridSystemData3.h"

using namespace CalfFluidEngine;

GridSystemData3::GridSystemData3() : GridSystemData3({ 0, 0, 0 }, { 1, 1, 1 }, { 0, 0, 0 })
{
}

CalfFluidEngine::GridSystemData3::GridSystemData3(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin)
{
	_velocity = std::make_shared<FaceCenteredGrid3>();
	_velocityIdx = 0;
	Resize(resolution, gridSpacing, origin);
}


GridSystemData3::~GridSystemData3()
{
}

void CalfFluidEngine::GridSystemData3::Resize(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin)
{
	_resolution = resolution;
	_gridSpacing = gridSpacing;
	_origin = origin;

	for (auto& data : _scalarDataList) {
		data->Resize(resolution, gridSpacing, origin);
	}
	for (auto& data : _vectorDataList) {
		data->Resize(resolution, gridSpacing, origin);
	}

}

BoundingBox3D CalfFluidEngine::GridSystemData3::GetBoundingBox() const
{
	return _velocity->GetBoundingBox();
}

const std::shared_ptr<ScalarGrid3>& CalfFluidEngine::GridSystemData3::ScalarDataAt(size_t idx) const
{
	return _scalarDataList[idx];
}

const std::shared_ptr<VectorGrid3>& CalfFluidEngine::GridSystemData3::VectorDataAt(size_t idx) const
{
	return _vectorDataList[idx];
}

size_t CalfFluidEngine::GridSystemData3::GetNumberOfScalarData() const
{
	return _scalarDataList.size();
}

size_t CalfFluidEngine::GridSystemData3::GetNnumberOfVectorData() const
{
	return _vectorDataList.size();
}

size_t CalfFluidEngine::GridSystemData3::AddScalarData(const std::shared_ptr<ScalarGridBuilder3>& builder, double initialVal)
{
	size_t attrIdx = _scalarDataList.size();
	_scalarDataList.push_back(
		builder->Build(GetResolution(), GetGridSpacing(), GetOrigin(), initialVal));
	return attrIdx;
}

size_t CalfFluidEngine::GridSystemData3::AddVectorData(const std::shared_ptr<VectorGridBuilder3>& builder, const Vector3D & initialVal)
{
	size_t attrIdx = _vectorDataList.size();
	_vectorDataList.push_back(
		builder->Build(GetResolution(), GetGridSpacing(), GetOrigin(), initialVal));
	return attrIdx;
}
