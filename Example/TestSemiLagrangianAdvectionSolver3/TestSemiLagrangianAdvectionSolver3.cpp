// SemiLagrangianAdvectionSolver3.cpp: 定义控制台应用程序的入口点。
//

#include <OutputForMatplotlib.h>
#include <Grid3.h>
#include <AdvectionSolver3.h>
using namespace CalfFluidEngine;

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
		return Vector3D::Distance(Vector3D(0.5, 0.5, 0), Vector3D(pt.x,pt.y,0)) - 0.25;
	});

	Array3<double> data(Vector3<size_t>(3, src.GetResolution().x, src.GetResolution().y));
	data.ParallelForEach([&](size_t t, size_t i, size_t j) {
		if (t < 2)
		{
			data(t, i, j) = src(i, j, 0)[t];
		}
	});
	OutputForMatplotlib o("TestSemiLagrangianAdvectionSolver3");
	o.SaveData(data, "src_#grid2.npy");

	auto Pos = src.Position();
	data.ParallelForEach([&data,&boundarySdf, Pos](size_t t, size_t i, size_t j) {
		if (t < 2)
		{
			data(t, i, j) =  boundarySdf.Sample(Pos(i, j, 0));
		}
	});
	o.SaveData(data, "bsd_#grid2.npy");

	SemiLagrangianAdvectionSolver3 solver;
	solver.Advect(src, flow, 0.1, &dst, boundarySdf);

	data.ParallelForEach([&](size_t t, size_t i, size_t j) {
		if (t < 2)
		{
			data(t, i, j) = dst(i, j, 0)[t];
		}
	});
	o.SaveData(data, "dst_#grid2.npy");
	return 0;
}

