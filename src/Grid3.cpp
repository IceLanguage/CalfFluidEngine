#include "Grid3.h"

using namespace CalfFluidEngine;

CalfFluidEngine::Grid3::Grid3()
{
}

CalfFluidEngine::Grid3::~Grid3()
{
}

const double & CalfFluidEngine::Grid3::operator()(size_t i, size_t j, size_t k) const
{
	return _data(i, j, k);
}

double & CalfFluidEngine::Grid3::operator()(size_t i, size_t j, size_t k)
{
	return _data(i, j, k);
}

std::function<Vector3D(size_t, size_t, size_t)> CalfFluidEngine::Grid3::GetCellCenterPosition() const
{
	Vector3D h = _gridSpacing;
	Vector3D o = _origin;
	return [h, o](size_t i, size_t j, size_t k) {
		return o + h * Vector3D(i + 0.5, j + 0.5, k + 0.5);
	};
}

void CalfFluidEngine::Grid3::setSizeParameters(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin)
{
	_resolution = resolution;
	_origin = origin;
	_gridSpacing = gridSpacing;

	Vector3D resolutionD = Vector3D(
		static_cast<double>(resolution.x),
		static_cast<double>(resolution.y),
		static_cast<double>(resolution.z)
	);

	_boundingBox = BoundingBox3D(origin, origin + gridSpacing * resolutionD);
}

CalfFluidEngine::ScalarGrid3::ScalarGrid3()
{
}

CalfFluidEngine::ScalarGrid3::~ScalarGrid3()
{
}

void CalfFluidEngine::ScalarGrid3::Resize(size_t resolutionX, size_t resolutionY, size_t resolutionZ, double gridSpacingX, double gridSpacingY, double gridSpacingZ, double originX, double originY, double originZ, double initialValue)
{
	Resize(Vector3<size_t>(resolutionX, resolutionY, resolutionZ),
		Vector3D(gridSpacingX, gridSpacingY, gridSpacingZ),
		Vector3D(originX, originY, originZ), initialValue);
}

void CalfFluidEngine::ScalarGrid3::Resize(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin, double initialValue)
{
	setSizeParameters(resolution, gridSpacing, origin);
	Vector3<size_t> size = GetDataSize();
	_data.Resize(size, initialValue);
}
