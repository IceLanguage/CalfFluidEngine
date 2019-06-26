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
