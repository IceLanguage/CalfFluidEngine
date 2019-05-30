#ifndef _CalfFluidEngine_Ray3_
#define _CalfFluidEngine_Ray3_
#include <Vector3.h>
namespace CalfFluidEngine {
	template <typename T>
	class Ray3
	{
	public:
		Ray3();
		virtual ~Ray3() {}
		Ray3(const Vector3<T>& newOrigin, const Vector3<T>& newDirection);
		Vector3<T> origin;
		Vector3<T> direction;
	};
	template<typename T>
	inline Ray3<T>::Ray3():Ray3(Vector3<T>::zero, Vector3<T>(1, 0, 0))
	{
	}
	template<typename T>
	inline Ray3<T>::Ray3(const Vector3<T>& newOrigin, const Vector3<T>& newDirection):
		origin(newOrigin),
		direction(Vector3<T>::Normalize(newDirection))
	{
	}

	typedef Ray3<float> Ray3F;
	typedef Ray3<double> Ray3D;
}
#endif