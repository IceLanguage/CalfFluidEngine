#include "GridBoundaryConditionSolver3.h"

using namespace CalfFluidEngine;

GridBoundaryConditionSolver3::GridBoundaryConditionSolver3()
{
}


GridBoundaryConditionSolver3::~GridBoundaryConditionSolver3()
{
}

void CalfFluidEngine::GridBoundaryConditionSolver3::UpdateCollider(const std::shared_ptr<Collider3>& newCollider, const Vector3<size_t>& gridSize, const Vector3D & gridSpacing, const Vector3D & gridOrigin)
{
	_collider = newCollider;
	_gridSize = gridSize;
	_gridSpacing = gridSpacing;
	_gridOrigin = gridOrigin;

	onColliderUpdated(gridSize, gridSpacing, gridOrigin);
}

CalfFluidEngine::GridFractionalBoundaryConditionSolver3::GridFractionalBoundaryConditionSolver3()
{
}

CalfFluidEngine::GridFractionalBoundaryConditionSolver3::~GridFractionalBoundaryConditionSolver3()
{
}

void CalfFluidEngine::GridFractionalBoundaryConditionSolver3::ConstrainVelocity(FaceCenteredGrid3 * velocity, unsigned int extrapolationDepth)
{

}

void CalfFluidEngine::GridFractionalBoundaryConditionSolver3::onColliderUpdated(const Vector3<size_t>& gridSize, const Vector3D & gridSpacing, const Vector3D & gridOrigin)
{
}
