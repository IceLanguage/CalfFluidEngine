#include "Grid3.h"
#include <array>
using namespace CalfFluidEngine;

CalfFluidEngine::Grid3::Grid3()
{
}

CalfFluidEngine::Grid3::~Grid3()
{
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

CalfFluidEngine::ScalarGrid3::ScalarGrid3() : 
	_linearSampler(LinearArraySampler3<double, double>(
	_data, Vector3D(1, 1, 1), Vector3D::zero)) 
{
}

CalfFluidEngine::ScalarGrid3::~ScalarGrid3()
{
}

const double & CalfFluidEngine::ScalarGrid3::operator()(size_t i, size_t j, size_t k) const
{
	return _data(i, j, k);
}

double & CalfFluidEngine::ScalarGrid3::operator()(size_t i, size_t j, size_t k)
{
	return _data(i, j, k);
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

Vector3D CalfFluidEngine::ScalarGrid3::GradientAtDataPoint(size_t i, size_t j, size_t k) const
{
	return Gradient3(_data, GetGridSpacing(), i, j, k);
}

Vector3D CalfFluidEngine::ScalarGrid3::Gradient(const Vector3D & x) const
{
	std::array<Vector3<size_t>, 8> indices;
	std::array<double, 8> weights;
	_linearSampler.GetCoordinatesAndWeights(x, &indices, &weights);
	Vector3D result;

	for (int i = 0; i < 8; ++i) {
		result += weights[i] *
			GradientAtDataPoint(indices[i].x, indices[i].y, indices[i].z);
	}

	return result;
}

CalfFluidEngine::VectorGrid3::VectorGrid3()
{
}

CalfFluidEngine::VectorGrid3::~VectorGrid3()
{
}

void CalfFluidEngine::VectorGrid3::Resize(size_t resolutionX, size_t resolutionY, size_t resolutionZ, double gridSpacingX, double gridSpacingY, double gridSpacingZ, double originX, double originY, double originZ, double initialValueX, double initialValueY, double initialValueZ)
{
	Resize(Vector3<size_t>(resolutionX, resolutionY, resolutionZ),
		Vector3D(gridSpacingX, gridSpacingY, gridSpacingZ),
		Vector3D(originX, originY, originZ),
		Vector3D(initialValueX, initialValueY, initialValueZ));
}

void CalfFluidEngine::VectorGrid3::Resize(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin, const Vector3D & initialValue)
{
	setSizeParameters(resolution, gridSpacing, origin);

	onResize(resolution, gridSpacing, origin, initialValue);
}

CalfFluidEngine::CellCenteredScalarGrid3::CellCenteredScalarGrid3()
{
}

CalfFluidEngine::CellCenteredScalarGrid3::CellCenteredScalarGrid3(size_t resolutionX, size_t resolutionY, size_t resolutionZ, double gridSpacingX, double gridSpacingY, double gridSpacingZ, double originX, double originY, double originZ, double initialValue)
{
	Resize(
		resolutionX,
		resolutionY,
		resolutionZ,
		gridSpacingX,
		gridSpacingY,
		gridSpacingZ,
		originX,
		originY,
		originZ,
		initialValue);
}

CalfFluidEngine::CellCenteredScalarGrid3::CellCenteredScalarGrid3(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin, double initialValue)
{
	Resize(resolution, gridSpacing, origin, initialValue);
}

Vector3<size_t> CalfFluidEngine::CellCenteredScalarGrid3::GetDataSize() const
{
	return GetResolution();
}

Vector3D CalfFluidEngine::CellCenteredScalarGrid3::GetDataOrigin() const
{
	return GetOrigin() + 0.5 * GetGridSpacing();
}

CalfFluidEngine::VertexCenteredScalarGrid3::VertexCenteredScalarGrid3()
{
}

CalfFluidEngine::VertexCenteredScalarGrid3::VertexCenteredScalarGrid3(size_t resolutionX, size_t resolutionY, size_t resolutionZ, double gridSpacingX, double gridSpacingY, double gridSpacingZ, double originX, double originY, double originZ, double initialValue)
{
	Resize(
		resolutionX,
		resolutionY,
		resolutionZ,
		gridSpacingX,
		gridSpacingY,
		gridSpacingZ,
		originX,
		originY,
		originZ,
		initialValue);
}

CalfFluidEngine::VertexCenteredScalarGrid3::VertexCenteredScalarGrid3(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin, double initialValue)
{
	Resize(resolution, gridSpacing, origin, initialValue);
}

Vector3<size_t> CalfFluidEngine::VertexCenteredScalarGrid3::GetDataSize() const
{
	if (GetResolution() != Vector3<size_t>(0, 0, 0)) {
		return GetResolution() + Vector3<size_t>(1, 1, 1);
	}
	else {
		return Vector3<size_t>(0, 0, 0);
	}
}

Vector3D CalfFluidEngine::VertexCenteredScalarGrid3::GetDataOrigin() const
{
	return GetOrigin();
}

CalfFluidEngine::FaceCenteredGrid3::FaceCenteredGrid3() :
_dataOriginU(0.0, 0.5, 0.5),
_dataOriginV(0.5, 0.0, 0.5),
_dataOriginW(0.5, 0.5, 0.0),
_uLinearSampler(LinearArraySampler3<double, double>(
_dataU, Vector3D(1, 1, 1), _dataOriginU)),
_vLinearSampler(LinearArraySampler3<double, double>(
_dataV, Vector3D(1, 1, 1), _dataOriginV)),
_wLinearSampler(LinearArraySampler3<double, double>(
_dataW, Vector3D(1, 1, 1), _dataOriginW))
{
}

CalfFluidEngine::FaceCenteredGrid3::FaceCenteredGrid3(size_t resolutionX, size_t resolutionY, size_t resolutionZ, double gridSpacingX, double gridSpacingY, double gridSpacingZ, double originX, double originY, double originZ, double initialValueU, double initialValueV, double initialValueW)

	: FaceCenteredGrid3(
		Vector3<size_t>(resolutionX, resolutionY, resolutionZ),
		Vector3D(gridSpacingX, gridSpacingY, gridSpacingZ),
		Vector3D(originX, originY, originZ),
		Vector3D(initialValueU, initialValueV, initialValueW))
{
}

CalfFluidEngine::FaceCenteredGrid3::FaceCenteredGrid3(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin, const Vector3D & initialValue):
	_uLinearSampler(LinearArraySampler3<double, double>(
		_dataU, Vector3D(1, 1, 1), _dataOriginU)),
	_vLinearSampler(LinearArraySampler3<double, double>(
		_dataV, Vector3D(1, 1, 1), _dataOriginV)),
	_wLinearSampler(LinearArraySampler3<double, double>(
		_dataW, Vector3D(1, 1, 1), _dataOriginW))
{
	Resize(resolution, gridSpacing, origin, initialValue);
}

double & CalfFluidEngine::FaceCenteredGrid3::u(size_t i, size_t j, size_t k)
{
	return _dataU(i, j, k);
}

const double & CalfFluidEngine::FaceCenteredGrid3::u(size_t i, size_t j, size_t k) const
{
	return _dataU(i, j, k);
}

double & CalfFluidEngine::FaceCenteredGrid3::v(size_t i, size_t j, size_t k)
{
	return _dataV(i, j, k);
}

const double & CalfFluidEngine::FaceCenteredGrid3::v(size_t i, size_t j, size_t k) const
{
	return _dataV(i, j, k);
}

double & CalfFluidEngine::FaceCenteredGrid3::w(size_t i, size_t j, size_t k)
{
	return _dataW(i, j, k);
}

const double & CalfFluidEngine::FaceCenteredGrid3::w(size_t i, size_t j, size_t k) const
{
	return _dataW(i, j, k);
}

void CalfFluidEngine::FaceCenteredGrid3::onResize(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin, const Vector3D & initialValue)
{
	if (resolution != Vector3<size_t>(0, 0, 0)) {
		_dataU.Resize(resolution + Vector3<size_t>(1, 0, 0), initialValue.x);
		_dataV.Resize(resolution + Vector3<size_t>(0, 1, 0), initialValue.y);
		_dataW.Resize(resolution + Vector3<size_t>(0, 0, 1), initialValue.z);
	}
	else {
		_dataU.Resize(Vector3<size_t>(0, 0, 0));
		_dataV.Resize(Vector3<size_t>(0, 0, 0));
		_dataW.Resize(Vector3<size_t>(0, 0, 0));
	}
	_dataOriginU = origin + 0.5 * Vector3D(0.0, gridSpacing.y, gridSpacing.z);
	_dataOriginV = origin + 0.5 * Vector3D(gridSpacing.x, 0.0, gridSpacing.z);
	_dataOriginW = origin + 0.5 * Vector3D(gridSpacing.x, gridSpacing.y, 0.0);

}

CalfFluidEngine::CollocatedVectorGrid3::CollocatedVectorGrid3():
	_linearSampler(LinearArraySampler3<Vector3D, double>(
	_data, Vector3D(1, 1, 1), Vector3D::zero))
{
}

CalfFluidEngine::CollocatedVectorGrid3::~CollocatedVectorGrid3()
{
}

const Vector3D & CalfFluidEngine::CollocatedVectorGrid3::operator()(size_t i, size_t j, size_t k) const
{
	return _data(i, j, k);
}

Vector3D & CalfFluidEngine::CollocatedVectorGrid3::operator()(size_t i, size_t j, size_t k)
{
	return _data(i, j, k);
}

void CalfFluidEngine::CollocatedVectorGrid3::onResize(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin, const Vector3D & initialValue)
{
	_data.Resize(GetDataSize(), initialValue);
}

CalfFluidEngine::CellCenteredVectorGrid3::CellCenteredVectorGrid3()
{
}

CalfFluidEngine::CellCenteredVectorGrid3::CellCenteredVectorGrid3(size_t resolutionX, size_t resolutionY, size_t resolutionZ, double gridSpacingX, double gridSpacingY, double gridSpacingZ, double originX, double originY, double originZ, double initialValueU, double initialValueV, double initialValueW)
{
	Resize(resolutionX, resolutionY, resolutionZ, gridSpacingX, gridSpacingY,
		gridSpacingZ, originX, originY, originZ, initialValueU,
		initialValueV, initialValueW);
}

CalfFluidEngine::CellCenteredVectorGrid3::CellCenteredVectorGrid3(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin, const Vector3D & initialValue)
{
	Resize(resolution, gridSpacing, origin, initialValue);
}

Vector3<size_t> CalfFluidEngine::CellCenteredVectorGrid3::GetDataSize() const
{
	return GetResolution();
}

Vector3D CalfFluidEngine::CellCenteredVectorGrid3::GetDataOrigin() const
{
	return GetOrigin() + 0.5 * GetGridSpacing();
}

CalfFluidEngine::VertexCenteredVectorGrid3::VertexCenteredVectorGrid3()
{
}

CalfFluidEngine::VertexCenteredVectorGrid3::VertexCenteredVectorGrid3(size_t resolutionX, size_t resolutionY, size_t resolutionZ, double gridSpacingX, double gridSpacingY, double gridSpacingZ, double originX, double originY, double originZ, double initialValueU, double initialValueV, double initialValueW)
{
	Resize(resolutionX, resolutionY, resolutionZ, gridSpacingX, gridSpacingY,
		gridSpacingZ, originX, originY, originZ, initialValueU,
		initialValueV, initialValueW);
}

CalfFluidEngine::VertexCenteredVectorGrid3::VertexCenteredVectorGrid3(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin, const Vector3D & initialValue)
{
	Resize(resolution, gridSpacing, origin, initialValue);
}

Vector3<size_t> CalfFluidEngine::VertexCenteredVectorGrid3::GetDataSize() const
{
	if (GetResolution() != Vector3<size_t>(0, 0, 0)) {
		return GetResolution() + Vector3<size_t>(1, 1, 1);
	}
	else {
		return Vector3<size_t>(0, 0, 0);
	}
}

Vector3D CalfFluidEngine::VertexCenteredVectorGrid3::GetDataOrigin() const
{
	return GetOrigin();
}

CalfFluidEngine::ScalarGridBuilder3::ScalarGridBuilder3()
{
}

CalfFluidEngine::ScalarGridBuilder3::~ScalarGridBuilder3()
{
}

CalfFluidEngine::VectorGridBuilder3::VectorGridBuilder3()
{
}

CalfFluidEngine::VectorGridBuilder3::~VectorGridBuilder3()
{
}
