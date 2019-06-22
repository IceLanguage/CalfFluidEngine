#ifndef _CalfFluidEngine_ArraySampler_
#define _CalfFluidEngine_ArraySampler_
#include <Array3.h>
#include <Math_utils.h>
#include <array>
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
			_invGridSpacing = Vector3<R>(static_cast<R>(1) / gridSpacing.x, static_cast<R>(1) / gridSpacing.y, static_cast<R>(1) / gridSpacing.z);
			_origin = gridOrigin;
			_accessor = std::make_shared<Array3<T>>(accessor);
		}
		LinearArraySampler3(const LinearArraySampler3& other)
		{
			_gridSpacing = other._gridSpacing;
			_invGridSpacing = other._invGridSpacing;
			_origin = other._origin;
			_accessor = std::make_shared<Array3<T>>(other._accessor);
		}
		T operator()(const Vector3<R>& x) const
		{
			size_t i, j, k;
			R fx, fy, fz;

			Vector3<R> normalizedX = (x - _origin) / _gridSpacing;

			Vector3<size_t> size = _accessor->Size();
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
				*_accessor(i, j, k),
				*_accessor(ip1, j, k),
				*_accessor(i, jp1, k),
				*_accessor(ip1, jp1, k),
				*_accessor(i, j, kp1),
				*_accessor(ip1, j, kp1),
				*_accessor(i, jp1, kp1),
				*_accessor(ip1, jp1, kp1),
				fx,
				fy,
				fz);
		}
		void GetCoordinatesAndWeights(
			const Vector3<R>& x,
			std::array<Vector3<size_t>, 8>* indices,
			std::array<R, 8>* weights) const
		{
			size_t i, j, k;
			R fx, fy, fz;

			const Vector3<R> normalizedX = (x - _origin) * _invGridSpacing;

			const size_t iSize = _accessor->Size().x;
			const size_t jSize = _accessor->Size().y;
			const size_t kSize = _accessor->Size().z;

			GetBarycentric(normalizedX.x, 0, iSize - 1, &i, &fx);
			GetBarycentric(normalizedX.y, 0, jSize - 1, &j, &fy);
			GetBarycentric(normalizedX.z, 0, kSize - 1, &k, &fz);

			const size_t ip1 = std::min(i + 1, iSize - 1);
			const size_t jp1 = std::min(j + 1, jSize - 1);
			const size_t kp1 = std::min(k + 1, kSize - 1);

			(*indices)[0] = Vector3<size_t>(i, j, k);
			(*indices)[1] = Vector3<size_t>(ip1, j, k);
			(*indices)[2] = Vector3<size_t>(i, jp1, k);
			(*indices)[3] = Vector3<size_t>(ip1, jp1, k);
			(*indices)[4] = Vector3<size_t>(i, j, kp1);
			(*indices)[5] = Vector3<size_t>(ip1, j, kp1);
			(*indices)[6] = Vector3<size_t>(i, jp1, kp1);
			(*indices)[7] = Vector3<size_t>(ip1, jp1, kp1);

			(*weights)[0] = (1 - fx) * (1 - fy) * (1 - fz);
			(*weights)[1] = fx * (1 - fy) * (1 - fz);
			(*weights)[2] = (1 - fx) * fy * (1 - fz);
			(*weights)[3] = fx * fy * (1 - fz);
			(*weights)[4] = (1 - fx) * (1 - fy) * fz;
			(*weights)[5] = fx * (1 - fy) * fz;
			(*weights)[6] = (1 - fx) * fy * fz;
			(*weights)[7] = fx * fy * fz;
		}
	private:
		Vector3<R> _gridSpacing;
		Vector3<R> _invGridSpacing;
		Vector3<R> _origin;
		std::shared_ptr<Array3<T>> _accessor;
	};
}
#endif