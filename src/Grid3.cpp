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
	resetSampler();
}

Vector3D CalfFluidEngine::ScalarGrid3::GetGradientAtDataPoint(size_t i, size_t j, size_t k) const
{
	return Gradient3(_data, GetGridSpacing(), i, j, k);
}

double CalfFluidEngine::ScalarGrid3::GetLaplacianAtDataPoint(size_t i, size_t j, size_t k) const
{
	return Laplacian3(_data, GetGridSpacing(), i, j, k);
}

Vector3D CalfFluidEngine::ScalarGrid3::Gradient(const Vector3D & x) const
{
	std::array<Vector3<size_t>, 8> indices;
	std::array<double, 8> weights;
	_linearSampler.GetCoordinatesAndWeights(x, &indices, &weights);
	Vector3D result;

	for (int i = 0; i < 8; ++i) {
		result += weights[i] *
			GetGradientAtDataPoint(indices[i].x, indices[i].y, indices[i].z);
	}

	return result;
}

double CalfFluidEngine::ScalarGrid3::Laplacian(const Vector3D & x) const
{
	std::array<Vector3<size_t>, 8> indices;
	std::array<double, 8> weights;
	_linearSampler.GetCoordinatesAndWeights(x, &indices, &weights);

	double result = 0.0;

	for (int i = 0; i < 8; ++i) {
		result += weights[i] * GetLaplacianAtDataPoint(indices[i].x, indices[i].y,
			indices[i].z);
	}

	return result;
}

double CalfFluidEngine::ScalarGrid3::Sample(const Vector3D & x) const
{
	return _sampler(x);
}

void CalfFluidEngine::ScalarGrid3::resetSampler()
{
	_linearSampler = LinearArraySampler3<double, double>(
		_data, GetGridSpacing(), GetDataOrigin());
	_sampler = _linearSampler.GetFunctor();
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

Array3<double>& CalfFluidEngine::FaceCenteredGrid3::UArray3()
{
	return _dataU;
}

const Array3<double>& CalfFluidEngine::FaceCenteredGrid3::UArray3() const
{
	return _dataU;
}

Array3<double>& CalfFluidEngine::FaceCenteredGrid3::VArray3()
{
	return _dataV;
}

const Array3<double>& CalfFluidEngine::FaceCenteredGrid3::VArray3() const
{
	return _dataV;
}

Array3<double>& CalfFluidEngine::FaceCenteredGrid3::WArray3()
{
	return _dataW;
}

const Array3<double>& CalfFluidEngine::FaceCenteredGrid3::WArray3() const
{
	return _dataW;
}

std::function<Vector3D(size_t, size_t, size_t)> CalfFluidEngine::FaceCenteredGrid3::UPosition() const
{
	Vector3D h = GetGridSpacing();

	return [this, h](size_t i, size_t j, size_t k) -> Vector3D {
		return _dataOriginU + h * Vector3D({ i, j, k });
	};
}

std::function<Vector3D(size_t, size_t, size_t)> CalfFluidEngine::FaceCenteredGrid3::VPosition() const
{
	Vector3D h = GetGridSpacing();

	return [this, h](size_t i, size_t j, size_t k) -> Vector3D {
		return _dataOriginV + h * Vector3D({ i, j, k });
	};
}

std::function<Vector3D(size_t, size_t, size_t)> CalfFluidEngine::FaceCenteredGrid3::WPosition() const
{
	Vector3D h = GetGridSpacing();

	return [this, h](size_t i, size_t j, size_t k) -> Vector3D {
		return _dataOriginW + h * Vector3D({ i, j, k });
	};
}

double CalfFluidEngine::FaceCenteredGrid3::GetDivergenceAtCellCenter(size_t i, size_t j, size_t k) const
{
	return Divergence3(_dataU, _dataV, _dataW, GetGridSpacing(), i, j, k);
}

Vector3D CalfFluidEngine::FaceCenteredGrid3::GetCurlAtCellCenter(size_t i, size_t j, size_t k) const
{
	const Vector3<size_t>& res = GetResolution();
	const Vector3D& gs = GetGridSpacing();

	Vector3D left = GetValueAtCellCenter((i > 0) ? i - 1 : i, j, k);
	Vector3D right = GetValueAtCellCenter((i + 1 < res.x) ? i + 1 : i, j, k);
	Vector3D down = GetValueAtCellCenter(i, (j > 0) ? j - 1 : j, k);
	Vector3D up = GetValueAtCellCenter(i, (j + 1 < res.y) ? j + 1 : j, k);
	Vector3D back = GetValueAtCellCenter(i, j, (k > 0) ? k - 1 : k);
	Vector3D front = GetValueAtCellCenter(i, j, (k + 1 < res.z) ? k + 1 : k);

	double Fx_ym = down.x;
	double Fx_yp = up.x;
	double Fx_zm = back.x;
	double Fx_zp = front.x;

	double Fy_xm = left.y;
	double Fy_xp = right.y;
	double Fy_zm = back.y;
	double Fy_zp = front.y;

	double Fz_xm = left.z;
	double Fz_xp = right.z;
	double Fz_ym = down.z;
	double Fz_yp = up.z;

	return Vector3D(
		0.5 * (Fz_yp - Fz_ym) / gs.y - 0.5 * (Fy_zp - Fy_zm) / gs.z,
		0.5 * (Fx_zp - Fx_zm) / gs.z - 0.5 * (Fz_xp - Fz_xm) / gs.x,
		0.5 * (Fy_xp - Fy_xm) / gs.x - 0.5 * (Fx_yp - Fx_ym) / gs.y);
}

Vector3D CalfFluidEngine::FaceCenteredGrid3::GetValueAtCellCenter(size_t i, size_t j, size_t k) const
{
	return 0.5 * Vector3D(_dataU(i, j, k) + _dataU(i + 1, j, k),
		_dataV(i, j, k) + _dataV(i, j + 1, k),
		_dataW(i, j, k) + _dataW(i, j, k + 1));
}

double CalfFluidEngine::FaceCenteredGrid3::Divergence(const Vector3D & x) const
{
	Vector3<size_t> res = GetResolution();
	size_t i, j, k;
	double fx, fy, fz;
	Vector3D cellCenterOrigin = GetOrigin() + 0.5 * GetGridSpacing();

	Vector3D normalizedX = (x - cellCenterOrigin) / GetGridSpacing();

	GetBarycentric(normalizedX.x, 0, res.x - 1, &i, &fx);
	GetBarycentric(normalizedX.y, 0, res.y - 1, &j, &fy);
	GetBarycentric(normalizedX.z, 0, res.z - 1, &k, &fz);

	std::array<Vector3<size_t>, 8> indices;
	std::array<double, 8> weights;
	indices[0] = Vector3<size_t>(i, j, k);
	indices[1] = Vector3<size_t>(i + 1, j, k);
	indices[2] = Vector3<size_t>(i, j + 1, k);
	indices[3] = Vector3<size_t>(i + 1, j + 1, k);
	indices[4] = Vector3<size_t>(i, j, k + 1);
	indices[5] = Vector3<size_t>(i + 1, j, k + 1);
	indices[6] = Vector3<size_t>(i, j + 1, k + 1);
	indices[7] = Vector3<size_t>(i + 1, j + 1, k + 1);

	weights[0] = (1.0 - fx) * (1.0 - fy) * (1.0 - fz);
	weights[1] = fx * (1.0 - fy) * (1.0 - fz);
	weights[2] = (1.0 - fx) * fy * (1.0 - fz);
	weights[3] = fx * fy * (1.0 - fz);
	weights[4] = (1.0 - fx) * (1.0 - fy) * fz;
	weights[5] = fx * (1.0 - fy) * fz;
	weights[6] = (1.0 - fx) * fy * fz;
	weights[7] = fx * fy * fz;

	double result = 0.0;

	for (int n = 0; n < 8; ++n) {
		result += weights[n] * GetDivergenceAtCellCenter(
			indices[n].x, indices[n].y, indices[n].z);
	}

	return result;
}

Vector3D CalfFluidEngine::FaceCenteredGrid3::Curl(const Vector3D & x) const
{
	Vector3<size_t> res = GetResolution();
	size_t i, j, k;
	double fx, fy, fz;
	Vector3D cellCenterOrigin = GetOrigin() + 0.5 * GetGridSpacing();

	Vector3D normalizedX = (x - cellCenterOrigin) / GetGridSpacing();

	GetBarycentric(normalizedX.x, 0, res.x - 1, &i, &fx);
	GetBarycentric(normalizedX.y, 0, res.y - 1, &j, &fy);
	GetBarycentric(normalizedX.z, 0, res.z - 1, &k, &fz);

	std::array<Vector3<size_t>, 8> indices;
	std::array<double, 8> weights;

	indices[0] = Vector3<size_t>(i, j, k);
	indices[1] = Vector3<size_t>(i + 1, j, k);
	indices[2] = Vector3<size_t>(i, j + 1, k);
	indices[3] = Vector3<size_t>(i + 1, j + 1, k);
	indices[4] = Vector3<size_t>(i, j, k + 1);
	indices[5] = Vector3<size_t>(i + 1, j, k + 1);
	indices[6] = Vector3<size_t>(i, j + 1, k + 1);
	indices[7] = Vector3<size_t>(i + 1, j + 1, k + 1);

	weights[0] = (1.0 - fx) * (1.0 - fy) * (1.0 - fz);
	weights[1] = fx * (1.0 - fy) * (1.0 - fz);
	weights[2] = (1.0 - fx) * fy * (1.0 - fz);
	weights[3] = fx * fy * (1.0 - fz);
	weights[4] = (1.0 - fx) * (1.0 - fy) * fz;
	weights[5] = fx * (1.0 - fy) * fz;
	weights[6] = (1.0 - fx) * fy * fz;
	weights[7] = fx * fy * fz;

	Vector3D result;

	for (int n = 0; n < 8; ++n) {
		result += weights[n] *
			GetCurlAtCellCenter(indices[n].x, indices[n].y, indices[n].z);
	}

	return result;
}

std::function<Vector3D(const Vector3D&)> CalfFluidEngine::FaceCenteredGrid3::Sampler() const
{
	return _sampler;
}

Vector3D CalfFluidEngine::FaceCenteredGrid3::Sample(const Vector3D & x) const
{
	return _sampler(x);
}

std::shared_ptr<VectorGrid3> CalfFluidEngine::FaceCenteredGrid3::Clone() const
{
	return std::shared_ptr<FaceCenteredGrid3>(
		new FaceCenteredGrid3(*this), 
		[](FaceCenteredGrid3* obj) { 
		delete obj; 
	});
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

	resetSampler();
}

void CalfFluidEngine::FaceCenteredGrid3::resetSampler()
{
	LinearArraySampler3<double, double> uSampler(_dataU,
		GetGridSpacing(), _dataOriginU);
	LinearArraySampler3<double, double> vSampler(_dataV,
		GetGridSpacing(), _dataOriginV);
	LinearArraySampler3<double, double> wSampler(_dataW,
		GetGridSpacing(), _dataOriginW);

	_uLinearSampler = uSampler;
	_vLinearSampler = vSampler;
	_wLinearSampler = wSampler;

	_sampler = [uSampler, vSampler, wSampler](const Vector3D& x) -> Vector3D {
		double u = uSampler(x);
		double v = vSampler(x);
		double w = wSampler(x);
		return Vector3D(u, v, w);
	};
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

double CalfFluidEngine::CollocatedVectorGrid3::GetDivergenceAtDataPoint(size_t i, size_t j, size_t k) const
{
	return Divergence3(_data, GetGridSpacing(), i, j, k);
}

Vector3D CalfFluidEngine::CollocatedVectorGrid3::GetCurlAtDataPoint(size_t i, size_t j, size_t k) const
{	
	return Curl3(_data, GetGridSpacing(), i, j, k);
}

double CalfFluidEngine::CollocatedVectorGrid3::Divergence(const Vector3D & x) const
{
	std::array<Vector3<size_t>, 8> indices;
	std::array<double, 8> weights;
	_linearSampler.GetCoordinatesAndWeights(x, &indices, &weights);

	double result = 0.0;

	for (int i = 0; i < 8; ++i) {
		result += weights[i] * GetDivergenceAtDataPoint(
			indices[i].x, indices[i].y, indices[i].z);
	}

	return result;
}

Vector3D CalfFluidEngine::CollocatedVectorGrid3::Curl(const Vector3D & x) const
{
	std::array<Vector3<size_t>, 8> indices;
	std::array<double, 8> weights;
	_linearSampler.GetCoordinatesAndWeights(x, &indices, &weights);

	Vector3D result;

	for (int i = 0; i < 8; ++i) {
		result += weights[i] * GetCurlAtDataPoint(
			indices[i].x, indices[i].y, indices[i].z);
	}

	return result;
}

Vector3D CalfFluidEngine::CollocatedVectorGrid3::Sample(const Vector3D & x) const
{
	return _sampler(x);
}

void CalfFluidEngine::CollocatedVectorGrid3::onResize(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & origin, const Vector3D & initialValue)
{
	_data.Resize(GetDataSize(), initialValue);
	resetSampler();
}

void CalfFluidEngine::CollocatedVectorGrid3::resetSampler()
{
	_linearSampler = LinearArraySampler3<Vector3D, double>(
		_data, GetGridSpacing(), GetDataOrigin());
	_sampler = _linearSampler.GetFunctor();
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
