#ifndef _CalfFluidEngine_ArraySampler_
#define _CalfFluidEngine_ArraySampler_
#include <Array3.h>
namespace CalfFluidEngine {
	//param T - The value type to sample.
	//param R - The real number type to return .
	template <typename T, typename R>
	class LinearArraySampler3 final
	{
	public:
		explicit LinearArraySampler(
			const Array3<T>& accessor,
			const Vector3<R>& gridSpacing,
			const Vector3<R>& gridOrigin)
		{
			_gridSpacing = gridSpacing;
			R one = static_cast<R>(1);
			_invGridSpacing = Vector3<R>(R / gridSpacing.x, R / gridSpacing.y, R / gridSpacing.z);
			_origin = gridOrigin;
			_accessor = accessor;
		}
		T operator()(const Vector3<R>& pt) const
		{

		}
	private:
		Vector3<R> _gridSpacing;
		Vector3<R> _invGridSpacing;
		Vector3<R> _origin;
		Array3<T> _accessor;
	};
}
#endif