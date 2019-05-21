#ifndef _CalfFluidEngine_Vector3_
#define _CalfFluidEngine_Vector3_
namespace CalfFluidEngine
{
	template <typename T>
	struct Vector3 final
	{
	public:
		Vector3() : x(0), y(0), z(0) {}
		Vector3(const float x, const float y, const float z)
			: x(x), y(y), z(z) {}

		float operator[](unsigned i) const
		{
			if (i == 0) return x;
			if (i == 1) return y;
			return z;
		}

		float& operator[](unsigned i)
		{
			if (i == 0) return x;
			if (i == 1) return y;
			return z;
		}

		void operator+=(const Vector3& v)
		{
			x += v.x;
			y += v.y;
			z += v.z;
		}

		Vector3 operator+(const Vector3& v) const
		{
			return Vector3(x + v.x, y + v.y, z + v.z);
		}

		void operator-=(const Vector3& v)
		{
			x -= v.x;
			y -= v.y;
			z -= v.z;
		}

		Vector3 operator-(const Vector3& v) const
		{
			return Vector3(x - v.x, y - v.y, z - v.z);
		}

		void operator*=(const float value)
		{
			x *= value;
			y *= value;
			z *= value;
		}

		Vector3 operator*(const float value) const
		{
			return Vector3(x * value, y * value, z * value);
		}

		void operator*=(const Vector3& v)
		{
			x *= v.x;
			y *= v.y;
			z *= v.z;
		}

		Vector3 operator*(const Vector3& v) const
		{
			return Vector3(x * v.x, y * v.y, z * v.z);
		}

		bool operator==(const Vector3& other) const
		{
			return
				x == other.x &&
				y == other.y &&
				z == other.z;
		}

		bool operator!=(const Vector3& other) const
		{
			return
				x != other.x ||
				y != other.y ||
				z != other.z;
		}

		float SquareMagnitude() const
		{
			return x * x + y * y + z * z;
		}

		float Magnitude() const
		{
			return sqrtf(SquareMagnitude());
		}

		void AddScaledVector(const Vector3& vector, float scale)
		{
			x += vector.x * scale;
			y += vector.y * scale;
			z += vector.z * scale;
		}

		void Normalize()
		{
			float l = Magnitude();
			if (l > 0)
			{
				(*this) *= ((float)1) / l;
			}
		}

		void Clear()
		{
			x = y = z = (T)0;
		}

		static Vector3 Cross(const Vector3& lhsV, const Vector3& rhsV)
		{
			return Vector3(
				lhsV.y * rhsV.z - lhsV.z * rhsV.y,
				lhsV.z * rhsV.x - lhsV.x * rhsV.z,
				lhsV.x * rhsV.y - lhsV.y * rhsV.x);
		}

		static float Dot(const Vector3& lhsV, const Vector3& rhsV)
		{
			return lhsV.x * rhsV.x + lhsV.y * rhsV.y + lhsV.z * rhsV.z;
		}

		static Vector3 Normalize(const Vector3& v)
		{
			float l = v.Magnitude();
			Vector3 res = zero;
			if (l > 0)
			{
				float reciprocal = ((float)1) / l;
				res = reciprocal * v;
			}
			return res;
		}

		float x, y, z;

		const static Vector3 right;
		const static Vector3 up;
		const static Vector3 forward;
		const static Vector3 zero;
		typedef Vector3<float> Vector3F;
	};

	template <typename T>
	Vector3<T> operator*(float value, const Vector3<T>& v)
	{
		return v * value;
	}

	typedef Vector3<float> Vector3F;

	typedef Vector3<double> Vector3D;
}
#endif