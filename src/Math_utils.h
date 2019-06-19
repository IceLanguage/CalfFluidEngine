#ifndef _CalfFluidEngine_Math_utils_
#define _CalfFluidEngine_Math_utils_
#include <algorithm>
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
#endif