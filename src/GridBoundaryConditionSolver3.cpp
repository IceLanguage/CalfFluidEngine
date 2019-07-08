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
	if (_colliderSignedDistance == nullptr) {
		_colliderSignedDistance = std::make_shared<CellCenteredScalarGrid3>();
	}
	_colliderSignedDistance->Resize(gridSize, gridSpacing, gridOrigin);

	if (GetCollider() != nullptr) {
		std::shared_ptr<Surface3> surface = GetCollider()->GetSurface();
		if (surface == nullptr) {
			surface = std::make_shared<Surface3>(surface);
		}

		_colliderSignedDistance->Fill([&](const Vector3D& pt) {
			return surface->SignedDistance(pt);
		});

		_colliderVel = std::shared_ptr<CustomVectorField3>
		(
			new CustomVectorField3(
			[&](const Vector3D& x) 
			{
				return GetCollider()->VelocityAt(x);
			}, gridSpacing.x),
			[](CustomVectorField3* obj) {
			delete obj;
		});
	}
	else {
		_colliderSignedDistance->Fill(kMaxD);

		_colliderVel = std::shared_ptr<CustomVectorField3>
			(
				new CustomVectorField3(
					[&](const Vector3D& x)
		{
			return Vector3D::zero;
		}, gridSpacing.x),
				[](CustomVectorField3* obj) {
			delete obj;
		});
	}
}
