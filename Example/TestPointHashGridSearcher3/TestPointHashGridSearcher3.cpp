// TestPointHashGridSearcher3.cpp: 定义控制台应用程序的入口点。
//


#include <PointNeighborSearcher3.h>
#include <PointGenerator3.h>
#include <BoundingBox3.h>
#include <OutputForMatplotlib.h>

using namespace CalfFluidEngine;
int main()
{
	OutputForMatplotlib o("TestPointHashGridSearcher3");
	std::vector<Vector3D> points;
	BccLatticePointGenerator pointsGenerator;
	BoundingBox3D bbox(
		Vector3D(0, 0, 0),
		Vector3D(1, 1, 1));
	double spacing = 0.1;

	pointsGenerator.Generate(bbox, spacing, &points);

	PointHashGridSearcher3 pointSearcher(4, 4, 4, 0.18);
	pointSearcher.Build(points);

	size_t width = 4, height = 4;
	std::vector<double> grid(width * height, 0.0);

	for (size_t j = 0; j < height; ++j) {
		for (size_t i = 0; i < width; ++i) {
			size_t key = pointSearcher.GetHashKeyFromBucketIndex(
				Vector3<size_t>(i,j,0));
			size_t value = pointSearcher.GetBuckets()[key].size();
			grid[i + width * j] += static_cast<double>(value);
		}
	}

	o.SaveData(grid, width, height, "data_#grid2.npy");
    return 0;
}

