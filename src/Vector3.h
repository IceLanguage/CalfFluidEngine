#ifndef _CalfFluidEngine_Vector3_
#define _CalfFluidEngine_Vector3_

#include <algorithm>

namespace CalfFluidEngine{
	template <typename T>
	struct Vector3 final{
	public:
		Vector3() : x(0), y(0), z(0) {}
		Vector3(const T x, const T y, const T z)
			: x(x), y(y), z(z) {}

		T operator[](unsigned int i) const{
			if (i == 0) return x;
			if (i == 1) return y;
			return z;
		}

		T& operator[](unsigned int i){
			if (i == 0) return x;
			if (i == 1) return y;
			return z;
		}

		void operator+=(const Vector3& v){
			x += v.x;
			y += v.y;
			z += v.z;
		}

		Vector3 operator+(const Vector3& v) const{
			return Vector3(x + v.x, y + v.y, z + v.z);
		}

		void operator-=(const Vector3& v){
			x -= v.x;
			y -= v.y;
			z -= v.z;
		}

		Vector3 operator-(const Vector3& v) const{
			return Vector3(x - v.x, y - v.y, z - v.z);
		}

		void operator*=(const T value){
			x *= value;
			y *= value;
			z *= value;
		}

		Vector3 operator*(const T value) const{
			return Vector3(x * value, y * value, z * value);
		}

		void operator/=(const T value){
			x /= value;
			y /= value;
			z /= value;
		}

		Vector3 operator/(const T value) const{
			return Vector3(x / value, y / value, z / value);
		}

		void operator*=(const Vector3& v){
			x *= v.x;
			y *= v.y;
			z *= v.z;
		}

		Vector3 operator*(const Vector3& v) const{
			return Vector3(x * v.x, y * v.y, z * v.z);
		}

		bool operator==(const Vector3& other) const{
			return
				x == other.x &&
				y == other.y &&
				z == other.z;
		}

		bool operator!=(const Vector3& other) const{
			return
				x != other.x ||
				y != other.y ||
				z != other.z;
		}

		T SquareMagnitude() const{
			return x * x + y * y + z * z;
		}

		T Magnitude() const{
			return sqrt(SquareMagnitude());
		}

		void AddScaledVector(const Vector3& vector, T scale){
			x += vector.x * scale;
			y += vector.y * scale;
			z += vector.z * scale;
		}

		void Normalize(){
			float l = Magnitude();
			if (l > 0){
				(*this) *= ((float)1) / l;
			}
		}

		void Clear(){
			x = y = z = (T)0;
		}

		bool IsSimilar(const Vector3& other, T epsilon = std::numeric_limits<T>::epsilon()) const {
			return (std::fabs(x - other.x) < epsilon) &&
				(std::fabs(y - other.y) < epsilon) &&
				(std::fabs(z - other.z) < epsilon);
		}

		static Vector3 Cross(const Vector3& lhsV, const Vector3& rhsV){
			return Vector3(
				lhsV.y * rhsV.z - lhsV.z * rhsV.y,
				lhsV.z * rhsV.x - lhsV.x * rhsV.z,
				lhsV.x * rhsV.y - lhsV.y * rhsV.x);
		}

		static float Dot(const Vector3& lhsV, const Vector3& rhsV){
			return lhsV.x * rhsV.x + lhsV.y * rhsV.y + lhsV.z * rhsV.z;
		}

		static Vector3 Normalize(const Vector3& v){
			float l = v.Magnitude();
			Vector3 res = zero;
			if (l > 0){
				float reciprocal = ((float)1) / l;
				res = reciprocal * v;
			}
			return res;
		}

		static T Distance(const Vector3& lhsV, const Vector3& rhsV)
		{
			return (lhsV - rhsV).Magnitude();
		}

		T x, y, z;

		const static Vector3 right;
		const static Vector3 up;
		const static Vector3 forward;
		const static Vector3 zero;
		
		typedef Vector3<float> Vector3F;
	};

	template <typename T>
	Vector3<T> operator-(const Vector3<T> & lhsV, const Vector3<T> & rhsV)
	{
		return Vector3<T>(lhsV.x - rhsV.x, lhsV.y - rhsV.y, lhsV.z - rhsV.z);
	}

	template <typename T>
	Vector3<T> operator*(float value, const Vector3<T>& v){
		return v * value;
	}

	template <typename T>
	Vector3<T> min(const Vector3<T>& a, const Vector3<T>& b) {
		return Vector3<T>(
			std::min(a.x, b.x), 
			std::min(a.y, b.y),
			std::min(a.z, b.z));
	}

	template <typename T>
	Vector3<T> max(const Vector3<T>& a, const Vector3<T>& b) {
		return Vector3<T>(
			std::max(a.x, b.x),
			std::max(a.y, b.y),
			std::max(a.z, b.z));
	}

	typedef Vector3<float> Vector3F;

	typedef Vector3<double> Vector3D;

	const Vector3D Vector3D::right = Vector3D(1, 0, 0);
	const Vector3D Vector3D::up = Vector3D(0, 1, 0);
	const Vector3D Vector3D::forward = Vector3D(0, 0, 1);
	const Vector3D Vector3D::zero = Vector3D(0, 0, 0);

	const Vector3F Vector3F::right = Vector3F(1, 0, 0);
	const Vector3F Vector3F::up = Vector3F(0, 1, 0);
	const Vector3F Vector3F::forward = Vector3F(0, 0, 1);
	const Vector3F Vector3F::zero = Vector3F(0, 0, 0);
}
#endif