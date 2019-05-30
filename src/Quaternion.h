#ifndef _CalfFluidEngine_Quaternion_
#define _CalfFluidEngine_Quaternion_

#include <Vector3.h>
namespace CalfFluidEngine {
	struct Quaternion
	{
	private:
		double data[4];
	public:
		Quaternion();
		Quaternion(const double r, const double i, const double j, const double k);
		double Get_r() const;
		double Get_i() const;
		double Get_j() const;
		double Get_k() const;
		void Set_r(double value);
		void Set_i(double value);
		void Set_j(double value);
		void Set_k(double value);
		void Normalize();
		void operator *=(const Quaternion &multiplier);
		void RotateByVector(const Vector3D& vector);
		void AddScaledVector(const Vector3D& vector, float scale);

	};
}
#endif