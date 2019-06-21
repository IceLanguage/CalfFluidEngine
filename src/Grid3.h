#ifndef _CalfFluidEngine_Grid3_
#define _CalfFluidEngine_Grid3_
#include <Vector3.h>
#include <BoundingBox3.h>
#include <Field3.h>
#include <vector>
namespace CalfFluidEngine {
	class Grid3
	{
	public:
		Grid3();
		virtual ~Grid3();
		const Vector3<size_t>& GetResolution() const { return _resolution; }
		const Vector3D& GetOrigin() const { return _origin; }
		const Vector3D& GetGridSpacing() const { return _gridSpacing; }
		const BoundingBox3D& GetBoundingBox() const { return _boundingBox; }
	protected:
		void setSizeParameters(
			const Vector3<size_t>& resolution, 
			const Vector3D& gridSpacing,
			const Vector3D& origin);
		std::vector<double> _data;
	private:
		Vector3<size_t> _resolution;
		Vector3D _gridSpacing = Vector3D(1.0, 1.0, 1.0);
		Vector3D _origin;
		BoundingBox3D _boundingBox = BoundingBox3D(Vector3D::zero, Vector3D::zero);
	};

	class ScalarGrid3 : public ScalarField3, public Grid3
	{
	public:
		ScalarGrid3();
		virtual ~ScalarGrid3();

		void Resize(
			size_t resolutionX,
			size_t resolutionY,
			size_t resolutionZ,
			double gridSpacingX = 1.0,
			double gridSpacingY = 1.0,
			double gridSpacingZ = 1.0,
			double originX = 0.0,
			double originY = 0.0,
			double originZ = 0.0,
			double initialValue = 0.0);

		void Resize(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing = Vector3D(1, 1, 1),
			const Vector3D& origin = Vector3D(),
			double initialValue = 0.0);
	};
}
#endif
