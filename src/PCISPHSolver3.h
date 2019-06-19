#ifndef _CalfFluidEngine_PCISPHSolver3_
#define _CalfFluidEngine_PCISPHSolver3_
#include <SphSystemSolver3.h>
namespace CalfFluidEngine {
	class PCISPHSolver3 : public SphSystemSolver3
	{
	public:
		PCISPHSolver3();
		virtual ~PCISPHSolver3();
	protected:
		virtual void accumulatePressureForce(double timeStepInSeconds) override;
	private:
		double computeDelta(double timeStepInSeconds);
		double computeBeta(double timeStepInSeconds);

		double _maxDensityErrorRatio = 0.01;
		unsigned int _maxNumberOfIterations = 5;
		std::vector<Vector3D> _tempPositions, _tempVelocities, _pressureForces;
		std::vector<double> _densityErrors;
	};
}
#endif