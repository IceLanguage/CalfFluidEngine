#include "GridFluidSolver3.h"
#include <memory>
CalfFluidEngine::GridFluidSolver3::GridFluidSolver3()
	: GridFluidSolver3({ 1, 1, 1 }, { 1, 1, 1 }, { 0, 0, 0 })
{
}

CalfFluidEngine::GridFluidSolver3::GridFluidSolver3(const Vector3<size_t>& resolution, const Vector3D & gridSpacing, const Vector3D & gridOrigin)
{
	_grids = std::make_shared<GridSystemData3>();
	_grids->Resize(resolution, gridSpacing, gridOrigin);
	_pressureSolver = std::make_shared<GridFractionalSinglePhasePressureSolver3>();
	_diffusionSolver = std::make_shared<GridBackwardEulerDiffusionSolver3>();
	_advectionSolver = std::make_shared<SemiLagrangianAdvectionSolver3>();
	SetIsUsingFixedTimeSteps(false);
}

CalfFluidEngine::GridFluidSolver3::~GridFluidSolver3()
{
}

void CalfFluidEngine::GridFluidSolver3::onInitialize()
{
	updateCollider(0.0);
	updateEmitter(0.0);
}

void CalfFluidEngine::GridFluidSolver3::onTimeStep(double timeIntervalInSeconds)
{
	timeStepStart(timeIntervalInSeconds);
	computeExternalForces(timeIntervalInSeconds);
	computeViscosity(timeIntervalInSeconds);
	computePressure(timeIntervalInSeconds);
	computeAdvection(timeIntervalInSeconds);
	timeStepEnd(timeIntervalInSeconds);
}

void CalfFluidEngine::GridFluidSolver3::computeExternalForces(double timeIntervalInSeconds)
{
	//Compute gravity
	if (_gravity.SquareMagnitude() > kEpsilonD) {
		std::shared_ptr<FaceCenteredGrid3> vel = _grids->GetVelocity();
		Array3<double>& u = vel->UArray3();
		Array3<double>& v = vel->VArray3();
		Array3<double>& w = vel->WArray3();

		if (std::abs(_gravity.x) > kEpsilonD) {
			u.ParallelForEach([&](size_t i, size_t j, size_t k) {
				u(i, j, k) += timeIntervalInSeconds * _gravity.x;
			});
		}

		if (std::abs(_gravity.y) > kEpsilonD) {
			v.ParallelForEach([&](size_t i, size_t j, size_t k) {
				v(i, j, k) += timeIntervalInSeconds * _gravity.y;
			});
		}

		if (std::abs(_gravity.z) > kEpsilonD) {
			w.ParallelForEach([&](size_t i, size_t j, size_t k) {
				w(i, j, k) += timeIntervalInSeconds * _gravity.z;
			});
		}

		applyBoundaryCondition();
	}
}

void CalfFluidEngine::GridFluidSolver3::computeViscosity(double timeIntervalInSeconds)
{
	if (_diffusionSolver != nullptr && _viscosityCoefficient > kEpsilonD) {
		std::shared_ptr<FaceCenteredGrid3> vel = _grids->GetVelocity();
		auto vel0 = std::dynamic_pointer_cast<FaceCenteredGrid3>(vel->Clone());

		_diffusionSolver->Solve(*vel0, _viscosityCoefficient,
			timeIntervalInSeconds, vel.get(),
			*GetColliderSignedDistance(), *GetFluidSignedDistance());
		applyBoundaryCondition();
	}
}

void CalfFluidEngine::GridFluidSolver3::computePressure(double timeIntervalInSeconds)
{
	if (_pressureSolver != nullptr) {
		auto vel = _grids->GetVelocity();
		auto vel0 = std::dynamic_pointer_cast<FaceCenteredGrid3>(vel->Clone());

		_pressureSolver->Solve(*vel0, timeIntervalInSeconds, vel.get(),
			*GetColliderSignedDistance());
		applyBoundaryCondition();
	}
}

void CalfFluidEngine::GridFluidSolver3::computeAdvection(double timeIntervalInSeconds)
{
	std::shared_ptr<FaceCenteredGrid3> vel = _grids->GetVelocity();
	if (_advectionSolver != nullptr) 
	{
		auto vel0 = std::dynamic_pointer_cast<FaceCenteredGrid3>(vel->Clone());
		_advectionSolver->Advect(*vel0, *vel0, timeIntervalInSeconds, vel.get(),
			*GetColliderSignedDistance());

		applyBoundaryCondition();
	}
}

std::shared_ptr<ScalarField3> CalfFluidEngine::GridFluidSolver3::GetColliderSignedDistance() const
{
	return _boundaryConditionSolver->GetColliderSignedDistance();
}

void CalfFluidEngine::GridFluidSolver3::applyBoundaryCondition()
{
	std::shared_ptr<FaceCenteredGrid3> vel = _grids->GetVelocity();

	if (vel != nullptr && _boundaryConditionSolver != nullptr) {
		unsigned int depth = static_cast<unsigned int>(std::ceil(_maxCfl));
		
	}
}

std::shared_ptr<ScalarField3> CalfFluidEngine::GridFluidSolver3::GetFluidSignedDistance() const
{
	return std::make_shared<ConstantScalarField3>(-kMaxD);
}

void CalfFluidEngine::GridFluidSolver3::timeStepStart(double timeStepInSeconds)
{
	updateCollider(timeStepInSeconds);
	updateEmitter(timeStepInSeconds);

	if (_boundaryConditionSolver != nullptr) {

	}

	applyBoundaryCondition();

	onTimeStepStart(timeStepInSeconds);
}

void CalfFluidEngine::GridFluidSolver3::timeStepEnd(double timeStepInSeconds)
{
	onTimeStepEnd(timeStepInSeconds);
}

void CalfFluidEngine::GridFluidSolver3::updateCollider(double timeIntervalInSeconds)
{
	if (_collider != nullptr) {
		_collider->Update(GetCurrentTime(), timeIntervalInSeconds);
	}
}

void CalfFluidEngine::GridFluidSolver3::updateEmitter(double timeIntervalInSeconds)
{
	if (_emitter != nullptr) {
		_emitter->Update(GetCurrentTime(), timeIntervalInSeconds);
	}
}
