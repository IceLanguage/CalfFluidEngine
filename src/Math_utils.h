#ifndef _CalfFluidEngine_Math_utils_
#define _CalfFluidEngine_Math_utils_
#include <algorithm>
inline double Clamp(double v, double min, double max)
{
	return std::max(std::min(v, 1.0), 0.0);
}

template<typename S, typename T>
inline S Lerp(const S& value0, const S& value1, T f) {
	return (1 - f) * value0 + f * value1;
}
#endif