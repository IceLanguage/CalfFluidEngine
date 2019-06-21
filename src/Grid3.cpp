#include "Grid3.h"

CalfFluidEngine::Grid3::Grid3()
{
}

CalfFluidEngine::Grid3::~Grid3()
{
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

	std::vector<double> grid;
	grid.resize(size.x * size.y * size.z, initialValue);
	grid._size = size;
	size_t iMin = std::min(size.x, _size.x);
	size_t jMin = std::min(size.y, _size.y);
	size_t kMin = std::min(size.z, _size.z);
	for (size_t k = 0; k < kMin; ++k) {
		for (size_t j = 0; j < jMin; ++j) {
			for (size_t i = 0; i < iMin; ++i) {
				grid(i, j, k) = at(i, j, k);
			}
		}
	}
	resetSampler();
}
