#ifndef _CalfFluidEngine_Math_utils_
#define _CalfFluidEngine_Math_utils_
#include <algorithm>
#include <Array3.h>

using namespace CalfFluidEngine;

template<typename T>
inline T Clamp(T v, T minV, T maxV)
{
	return std::max(std::min(v, maxV), minV);
}

template<typename S, typename T>
inline S Lerp(const S& value0, const S& value1, T f) {
	return (1 - f) * value0 + f * value1;
}

template <typename T>
inline T AbsMax(T x, T y) {
	return (x*x > y*y) ? x : y;
}

Vector3D Gradient3(
	const Array3<double>& data,
	const Vector3D& gridSpacing,
	size_t i,
	size_t j,
	size_t k)
{
	const Vector3<size_t> ds = data.Size();
	double left = data((i > 0) ? i - 1 : i, j, k);
	double right = data((i + 1 < ds.x) ? i + 1 : i, j, k);
	double down = data(i, (j > 0) ? j - 1 : j, k);
	double up = data(i, (j + 1 < ds.y) ? j + 1 : j, k);
	double back = data(i, j, (k > 0) ? k - 1 : k);
	double front = data(i, j, (k + 1 < ds.z) ? k + 1 : k);

	return 0.5 * Vector3D(right - left, up - down, front - back) / gridSpacing;
}
#endif