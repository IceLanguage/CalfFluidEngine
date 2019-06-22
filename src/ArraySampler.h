#ifndef _CalfFluidEngine_ArraySampler_
#define _CalfFluidEngine_ArraySampler_
#include <Array3.h>
#include <Math_utils.h>
namespace CalfFluidEngine {
	//param T - The value type to sample.
	//param R - The real number type to return .
	template <typename T, typename R>
	class LinearArraySampler3 final
	{
	public:
		explicit LinearArraySampler3(
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
		LinearArraySampler3(const LinearArraySampler3& other)
		{
			_gridSpacing = other._gridSpacing;
			_invGridSpacing = other._invGridSpacing;
			_origin = other._origin;
			_accessor = other._accessor;
		}
		T operator()(const Vector3<R>& x) const
		{
			size_t i, j, k;
			R fx, fy, fz;

			Vector3<R> normalizedX = (x - _origin) / _gridSpacing;

			Vector3<size_t> size = _accessor.size();
			size_t iSize = size.x;
			size_t jSize = size.y;
			size_t kSize = size.z;

			GetBarycentric(normalizedX.x, 0, iSize - 1, &i, &fx);
			GetBarycentric(normalizedX.y, 0, jSize - 1, &j, &fy);
			GetBarycentric(normalizedX.z, 0, kSize - 1, &k, &fz);

			size_t ip1 = std::min(i + 1, iSize - 1);
			size_t jp1 = std::min(j + 1, jSize - 1);
			size_t kp1 = std::min(k + 1, kSize - 1);

			return TrilinearLerp(
				_accessor(i, j, k),
				_accessor(ip1, j, k),
				_accessor(i, jp1, k),
				_accessor(ip1, jp1, k),
				_accessor(i, j, kp1),
				_accessor(ip1, j, kp1),
				_accessor(i, jp1, kp1),
				_accessor(ip1, jp1, kp1),
				fx,
				fy,
				fz);
		}
	private:
		Vector3<R> _gridSpacing;
		Vector3<R> _invGridSpacing;
		Vector3<R> _origin;
		Array3<T>& _accessor;
	};
}
#endif