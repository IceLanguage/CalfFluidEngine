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

		virtual std::function<Vector3D(const Vector3D&)> Sampler() const;
	};

	class ScalarField3 : public Field3 {
	public:
		ScalarField3();
		virtual ~ScalarField3();

		//**********************************************
		//Returns sampled value at given position ;
		//**********************************************
		virtual double Sample(const Vector3D& x) const = 0;

		//**********************************************
		//Returns gradient vector at given position ;
		//**********************************************
		virtual Vector3D Gradient(const Vector3D& x) const = 0;

		//**********************************************
		//Returns Laplacian at given position ;
		//**********************************************
		virtual double Laplacian(const Vector3D& x) const = 0;

		virtual std::function<double(const Vector3D&)> Sampler() const;
	private:
		std::function<double(const Vector3D&)> _sampler;
	};

	class ConstantVectorField3 final : public VectorField3 {
	public:
		explicit ConstantVectorField3(const Vector3D& value);
		Vector3D Sample(const Vector3D& x) const override;
		double Divergence(const Vector3D& x) const override;
		Vector3D Curl(const Vector3D& x) const override;
	private:
		Vector3D _value;
	};
}


#endif