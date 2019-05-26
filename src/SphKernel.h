#ifndef _CalfFluidEngine_SphKernel_
#define _CalfFluidEngine_SphKernel_
#include <Vector3.h>
namespace CalfFluidEngine {
	class SphStandardKernel3 final
	{
	public:
		SphStandardKernel3();
		explicit SphStandardKernel3(double kernelRadius);
		~SphStandardKernel3();
		double operator()(double distance) const;
		Vector3D Gradient(const Vector3D& point) const;
		Vector3D Gradient(double distance, const Vector3D& directionToParticle) const;
		double Laplacian(double distance) const;
	private:
		double firstDerivative(double distance) const;
		double secondDerivative(double distance) const;
		//Kernel radius.
		double h;

		//Square of the kernel radius.
		double h2;

		double h3;
	};

	class SphSpikyKernel3 final
	{
	public:
		SphSpikyKernel3();
		explicit SphSpikyKernel3(double kernelRadius);
		~SphSpikyKernel3();
		double operator()(double distance) const;
		Vector3D Gradient(const Vector3D& point) const;
		Vector3D Gradient(double distance, const Vector3D& directionToParticle) const;
		double Laplacian(double distance) const;
	private:
		double firstDerivative(double distance) const;
		double secondDerivative(double distance) const;
		//Kernel radius.
		double h;

		//Square of the kernel radius.
		double h2;

		double h3;
	};
}
#endif