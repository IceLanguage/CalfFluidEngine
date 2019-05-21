#ifndef _CalfFluidEngine_Field3_
#define _CalfFluidEngine_Field3_

#include <Vector3.h>
#include <functional>

namespace CalfFluidEngine{
	class Field3{
	public:
		Field3();
		virtual ~Field3();
	};

	class VectorField3 : public Field3 {
	public:
		VectorField3();

		virtual ~VectorField3();

		//**********************************************
		//Returns sampled value at given position ;
		//**********************************************
		virtual Vector3D Sample(const Vector3D& x) const = 0;

		//**********************************************
		//Returns divergence at given position;
		//**********************************************
		virtual double Divergence(const Vector3D& x) const = 0;

		//**********************************************
		//Returns curl at given position ;
		//**********************************************
		virtual Vector3D Curl(const Vector3D& x) const = 0;

		//! Returns sampler function object.
		virtual std::function<Vector3D(const Vector3D&)> Sampler() const;
	};
}
#endif