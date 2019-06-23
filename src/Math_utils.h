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
inline S BilinearLerp(const S& f00, const S& f10, const S& f01, const S& f11, T tx,
	T ty)
{
	return Lerp(
		Lerp(f00, f10, tx),
		Lerp(f01, f11, tx),
		ty);
}

//Computes trilinear interpolation.
template <typename S, typename T>
inline S TrilinearLerp(const S& f000, const S& f100, const S& f010, const S& f110,
	const S& f001, const S& f101, const S& f011, const S& f111,
	T tx, T ty, T tz)
{
	return Lerp(
		BilinearLerp(f000, f100, f010, f110, tx, ty),
		BilinearLerp(f001, f101, f011, f111, tx, ty),
		fz);
}

template <typename T>
inline T AbsMax(T x, T y) {
	return (x*x > y*y) ? x : y;
}

// Gets the barycentric coordinate.
template <class T>
inline void GetBarycentric(T x, size_t iLow, size_t iHigh, size_t* i, T* f)
{
	T s = std::floor(x);
	*i = static_cast<size_t>(s);

	iLow -= iLow;
	iHigh -= iLow;

	if (iLow == iHigh) {
		*i = iLow;
		*f = 0;
	}
	else if (*i < iLow) {
		*i = iLow;
		*f = 0;
	}
	else if (*i > iHigh - 1) {
		*i = iHigh - 1;
		*f = 1;
	}
	else {
		*f = static_cast<T>(x - s);
	}

	*i += iLow;
}

inline Vector3D Gradient3(
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

inline double Divergence3(
	const Array3<Vector3D>& data,
	const Vector3D& gridSpacing,
	size_t i,
	size_t j,
	size_t k)
{
	const Vector3<size_t> ds = data.Size();

	double left = data((i > 0) ? i - 1 : i, j, k).x;
	double right = data((i + 1 < ds.x) ? i + 1 : i, j, k).x;
	double down = data(i, (j > 0) ? j - 1 : j, k).y;
	double up = data(i, (j + 1 < ds.y) ? j + 1 : j, k).y;
	double back = data(i, j, (k > 0) ? k - 1 : k).z;
	double front = data(i, j, (k + 1 < ds.z) ? k + 1 : k).z;

	return 0.5 * (right - left) / gridSpacing.x
		+ 0.5 * (up - down) / gridSpacing.y
		+ 0.5 * (front - back) / gridSpacing.z;
}

inline double Divergence3(
	const Array3<double>& dataU,
	const Array3<double>& dataV,
	const Array3<double>& dataW,
	const Vector3D& gridSpacing,
	size_t i,
	size_t j,
	size_t k)
{
	double leftU = dataU(i, j, k);
	double rightU = dataU(i + 1, j, k);
	double bottomV = dataV(i, j, k);
	double topV = dataV(i, j + 1, k);
	double backW = dataW(i, j, k);
	double frontW = dataW(i, j, k + 1);

	return
		(rightU - leftU) / gridSpacing.x +
		(topV - bottomV) / gridSpacing.y +
		(frontW - backW) / gridSpacing.z;
}

inline Vector3D Curl3(const Array3<Vector3D>& data,
	const Vector3D& gridSpacing,
	size_t i,
	size_t j,
	size_t k)
{
	const Vector3<size_t> ds = data.Size();

	Vector3D left = data((i > 0) ? i - 1 : i, j, k);
	Vector3D right = data((i + 1 < ds.x) ? i + 1 : i, j, k);
	Vector3D down = data(i, (j > 0) ? j - 1 : j, k);
	Vector3D up = data(i, (j + 1 < ds.y) ? j + 1 : j, k);
	Vector3D back = data(i, j, (k > 0) ? k - 1 : k);
	Vector3D front = data(i, j, (k + 1 < ds.z) ? k + 1 : k);

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
		0.5 * (Fz_yp - Fz_ym) / gridSpacing.y - 0.5 * (Fy_zp - Fy_zm) / gridSpacing.z,
		0.5 * (Fx_zp - Fx_zm) / gridSpacing.z - 0.5 * (Fz_xp - Fz_xm) / gridSpacing.x,
		0.5 * (Fy_xp - Fy_xm) / gridSpacing.x - 0.5 * (Fx_yp - Fx_ym) / gridSpacing.y);
}
#endif