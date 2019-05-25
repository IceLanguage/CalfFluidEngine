#ifndef _CalfFluidEngine_SphSystemData3_
#define _CalfFluidEngine_SphSystemData3_
#include <ParticleSystemSolver3.h>
namespace CalfFluidEngine {
	class SphSystemData3 :public ParticleSystemData3
	{
	public:
		SphSystemData3();
		explicit SphSystemData3(size_t numberOfParticles);
		~SphSystemData3();
		std::vector<double> GetDensities() const;
		std::vector<double> GetPressures() const;
		std::vector<double> GetDensities();
		std::vector<double> GetPressures();

		Vector3D Interpolate(
			const Vector3D& origin,
			const std::vector<Vector3D>& values) const;
		void UpdateDensities();

		//Returns sum of kernel function evaluation for each nearby particle.
		double SumOfKernelNearby(const Vector3D& origin) const;
	protected:
		size_t _pressureIdx;
		size_t _densityIdx;
		double _kernelRadius;
	};
}
#endif