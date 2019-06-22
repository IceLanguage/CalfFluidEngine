#ifndef _CalfFluidEngine_Math_utils_
#define _CalfFluidEngine_Math_utils_
#include <algorithm>
#include <Array3.h>

using namespace CalfFluidEngine;

template<typename T>
inline T Clamp(T v, T minV, T maxV)
{
	if (v < minV) {
		return minV;
	}
	else if (v > maxV) {
		return maxV;
	}
	else {
		return v;
	}
}

template<typename S, typename T>
inline S Lerp(const S& value0, const S& value1, T f) {
	return (1 - f) * value0 + f * value1;
}

//Computes bilinear interpolation.
template <typename S, typename T>
inline S Bilinearlerp(const S& f00, const S& f10, const S& f01, const S& f11, T tx,
	T ty)
{
	return Lerp(
		Lerp(f00, f10, tx),
		Lerp(f01, f11, tx),
		ty);
}

//Computes trilinear interpolation.
template <typename S, typename T>
inline S Trilinearlerp(const S& f000, const S& f100, const S& f010, const S& f110,
	const S& f001, const S& f101, const S& f011, const S& f111,
	T tx, T ty, T tz)
{
	return Lerp(
		Bilinearlerp(f000, f100, f010, f110, tx, ty),
		Bilinearlerp(f001, f101, f011, f111, tx, ty),
		fz);
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