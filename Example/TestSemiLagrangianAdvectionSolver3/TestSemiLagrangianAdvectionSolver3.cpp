// SemiLagrangianAdvectionSolver3.cpp: 定义控制台应用程序的入口点。
//

#include <OutputForMatplotlib.h>
#include <Grid3.h>
#include <AdvectionSolver3.h>
using namespace CalfFluidEngine;
class CustomScalarField3 final : public ScalarField3 {
public:
	CustomScalarField3(
		const std::function<double(const Vector3D&)>& customFunction,
		double derivativeResolution = 1e-3);
	double Sample(const Vector3D& x) const override;
	std::function<double(const Vector3D&)> Sampler() const override;
	Vector3D Gradient(const Vector3D& x) const override;
	double Laplacian(const Vector3D& x) const override;
private:
	std::function<double(const Vector3D&)> _customFunction;
	double _resolution = 1e-3;
};
int main()
{
	CellCenteredVectorGrid3 src(200, 200, 1, 1.0 / 200.0, 1.0 / 200.0);
	CellCenteredVectorGrid3 dst(200, 200, 1, 1.0 / 200.0, 1.0 / 200.0);
	src.Fill([&](const Vector3D& pt) -> Vector3D {
		return {
			0.5 * (std::sin(15 * pt.x) + 1.0),
			0.5 * (std::sin(15 * pt.y) + 1.0),0 };
	});

	ConstantVectorField3 flow(Vector3D(1.0, 1.0, 0.0));
	CustomScalarField3 boundarySdf([](const Vector3D& pt) {
		return Vector3D::Distance(Vector3D(0.5, 0.5, 0), pt) - 0.25;
	});

	Array3<double> data(Vector3<size_t>(3, src.GetResolution().x, src.GetResolution().y));
	data.ParallelForEach([&](size_t i, size_t j, size_t k) {
		if (i < 2)
		{
			data(i, j, k) = src(0, j, k)[i];
		}
	});
	OutputForMatplotlib o("TestSemiLagrangianAdvectionSolver3");
	o.SaveData(data, "src_#grid2.npy");

	/*SemiLagrangianAdvectionSolver3 solver;
	solver.Advect(src, flow, 0.1, &dst, boundarySdf);*/

	data.ParallelForEach([&](size_t i, size_t j, size_t k) {
		if (i < 2)
		{
			data(i, j, k) = dst(0, j, k)[i];
		}
	});
	o.SaveData(data, "dst_#grid2.npy");
	return 0;
}

CustomScalarField3::CustomScalarField3(
	const std::function<double(const Vector3D&)>& customFunction,
	double derivativeResolution) :
	_customFunction(customFunction),
	_resolution(derivativeResolution)
{
}

double CustomScalarField3::Sample(const Vector3D & x) const
{
	return _customFunction(x);
}

std::function<double(const Vector3D&)> CustomScalarField3::Sampler() const
{
	return _customFunction;
}

Vector3D CustomScalarField3::Gradient(const Vector3D & x) const
{
	double left
		= _customFunction(x - Vector3D(0.5 * _resolution, 0.0, 0.0));
	double right
		= _customFunction(x + Vector3D(0.5 * _resolution, 0.0, 0.0));
	double bottom
		= _customFunction(x - Vector3D(0.0, 0.5 * _resolution, 0.0));
	double top
		= _customFunction(x + Vector3D(0.0, 0.5 * _resolution, 0.0));
	double back
		= _customFunction(x - Vector3D(0.0, 0.0, 0.5 * _resolution));
	double front
		= _customFunction(x + Vector3D(0.0, 0.0, 0.5 * _resolution));

	return Vector3D(
		(right - left) / _resolution,
		(top - bottom) / _resolution,
		(front - back) / _resolution);
}

double CustomScalarField3::Laplacian(const Vector3D & x) const
{
	double center = _customFunction(x);
	double left
		= _customFunction(x - Vector3D(0.5 * _resolution, 0.0, 0.0));
	double right
		= _customFunction(x + Vector3D(0.5 * _resolution, 0.0, 0.0));
	double bottom
		= _customFunction(x - Vector3D(0.0, 0.5 * _resolution, 0.0));
	double top
		= _customFunction(x + Vector3D(0.0, 0.5 * _resolution, 0.0));
	double back
		= _customFunction(x - Vector3D(0.0, 0.0, 0.5 * _resolution));
	double front
		= _customFunction(x + Vector3D(0.0, 0.0, 0.5 * _resolution));

	return (left + right + bottom + top + back + front - 6.0 * center)
		/ (_resolution * _resolution);
}
