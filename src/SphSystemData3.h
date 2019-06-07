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
		const std::vector<double>& GetDensities() const;
		const std::vector<double>& GetPressures() const;
		std::vector<double>& GetDensities();
		std::vector<double>& GetPressures();
		double GetTargetSpacing() const { return _targetSpacing; }
		void SetTargetSpacing(double spacing);
		Vector3D Interpolate(
			const Vector3D& origin,
			const std::vector<Vector3D>& values) const;
		void UpdateDensities();

		//Returns sum of kernel function evaluation for each nearby particle.
		double SumOfKernelNearby(const Vector3D& origin) const;

		Vector3D GradientAt(
			size_t i,
			const std::vector<double>& values) const;

		double LaplacianAt(
			size_t i,
			const std::vector<double>& values) const;

		double GetDensity() const;
		double GetKernelRadius() const;

		void SetParticleMass(double newMass) override;
	private:
		//! Computes the mass based on the target density and spacing.
		void computeMass();
	protected:
		size_t _pressureIdx;
		size_t _densityIdx;
		double _kernelRadius;
		double _density = 1000.0;
		double _targetSpacing = 0.1;

		//! Relative radius of SPH kernel.
		//! SPH kernel radius divided by target spacing.
		double _kernelRadiusOverTargetSpacing = 1.8;
	};
}
#endif