#ifndef _CalfFluidEngine_SphKernel_
#define _CalfFluidEngine_SphKernel_
namespace CalfFluidEngine {
	class SphStandardKernel3 final
	{
	public:
		SphStandardKernel3();
		explicit SphStandardKernel3(double kernelRadius);
		~SphStandardKernel3();
		double operator()(double distance) const;
		
	private:
		double firstDerivative(double distance) const;
		//Kernel radius.
		double h;

		//Square of the kernel radius.
		double h2;

		double h3;
	};
}
#endif