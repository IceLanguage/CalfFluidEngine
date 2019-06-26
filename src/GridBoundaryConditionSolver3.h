#ifndef _CalfFluidEngine_GridBoundaryConditionSolver3_
#define _CalfFluidEngine_GridBoundaryConditionSolver3_
#include <Collider3.h>
#include <Grid3.h>
namespace CalfFluidEngine {
	class GridBoundaryConditionSolver3
	{
	public:
		GridBoundaryConditionSolver3();
		virtual ~GridBoundaryConditionSolver3();
		void UpdateCollider(
			const std::shared_ptr<Collider3>& newCollider,
			const Vector3<size_t>& gridSize,
			const Vector3D& gridSpacing,
			const Vector3D& gridOrigin);

		//**********************************************
		//Returns the signed distance field of the collider.
		//**********************************************
		virtual std::shared_ptr<ScalarField3> GetColliderSignedDistance() const = 0;

		//**********************************************
		//Returns the velocity field of the collider.
		//**********************************************
		virtual std::shared_ptr<VectorField3> GetColliderVelocityField() const = 0;
	protected:

		//**********************************************
		//Invoked when a new collider is set.
		//**********************************************
		virtual void onColliderUpdated(
			const Vector3<size_t>& gridSize,
			const Vector3D& gridSpacing,
			const Vector3D& gridOrigin) = 0;
	private:
		std::shared_ptr<Collider3> _collider;
		Vector3<size_t> _gridSize;
		Vector3D _gridSpacing;
		Vector3D _gridOrigin;
	};
}
#endif