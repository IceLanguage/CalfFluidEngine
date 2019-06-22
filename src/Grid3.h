#ifndef _CalfFluidEngine_Grid3_
#define _CalfFluidEngine_Grid3_
#include <Vector3.h>
#include <BoundingBox3.h>
#include <Field3.h>
#include <Array3.h>
#include <memory>
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
		const double& operator()(size_t i, size_t j, size_t k) const;
		double& operator()(size_t i, size_t j, size_t k);
		std::function<Vector3D(size_t, size_t, size_t)> GetCellCenterPosition() const;
	protected:
		void setSizeParameters(
			const Vector3<size_t>& resolution, 
			const Vector3D& gridSpacing,
			const Vector3D& origin);
		Array3<double> _data;
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

		//**********************************************
		//Returns the size of the grid data.
		//equal to the grid resolution if the data is not stored at cell-center.
		//**********************************************
		virtual Vector3<size_t> GetDataSize() const = 0;

		//**********************************************
		//Returns the origin of the grid data.
		//**********************************************
		virtual Vector3D GetDataOrigin() const = 0;

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
			const Vector3D& origin = Vector3D::zero,
			double initialValue = 0.0);

		Vector3D GradientAtDataPoint(size_t i, size_t j, size_t k) const;
		virtual Vector3D Gradient(const Vector3D& x) const override;
	};

	class VectorGrid3 : public VectorField3, public Grid3 
	{
	public:
		VectorGrid3();
		virtual ~VectorGrid3();
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
			double initialValueX = 0.0,
			double initialValueY = 0.0,
			double initialValueZ = 0.0);

		void Resize(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing = Vector3D(1, 1, 1),
			const Vector3D& origin = Vector3D::zero,
			const Vector3D& initialValue = Vector3D::zero);
	protected:
		//**********************************************
		//Invoked when the resizing happens.
		//The overriding class should allocate the internal storage based on its data layout scheme.
		//**********************************************
		virtual void onResize(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing,
			const Vector3D& origin,
			const Vector3D& initialValue) = 0;
	};

	// the class defines the data point at the
	// center of a grid cell
	class CellCenteredScalarGrid3 final : public ScalarGrid3
	{
	public:
		CellCenteredScalarGrid3();
		CellCenteredScalarGrid3(
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
		CellCenteredScalarGrid3(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing = Vector3D(1.0, 1.0, 1.0),
			const Vector3D& origin = Vector3D::zero,
			double initialValue = 0.0);
		virtual Vector3<size_t> GetDataSize() const override;
		virtual Vector3D GetDataOrigin() const override;
	};

	//the class defines the data point at the grid vertices (corners). 
	//Thus, A x B x C grid resolution will have (A+1) x (B+1) x (C+1) data points.
	class VertexCenteredScalarGrid3 final : public ScalarGrid3 {
	public:
		VertexCenteredScalarGrid3();
		VertexCenteredScalarGrid3(
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
		VertexCenteredScalarGrid3(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing = Vector3D(1.0, 1.0, 1.0),
			const Vector3D& origin = Vector3D::zero,
			double initialValue = 0.0);
		virtual Vector3<size_t> GetDataSize() const override;
		virtual Vector3D GetDataOrigin() const override;
	};


	//This class implements face-centered grid which is also known as
	//marker-and-cell (MAC) or staggered grid. This vector grid stores 
	//each vector component at face center.
	class FaceCenteredGrid3 final : public VectorGrid3 {
	public:
		FaceCenteredGrid3();
		FaceCenteredGrid3(size_t resolutionX, size_t resolutionY,
			size_t resolutionZ, double gridSpacingX = 1.0,
			double gridSpacingY = 1.0, double gridSpacingZ = 1.0,
			double originX = 0.0, double originY = 0.0,
			double originZ = 0.0, double initialValueU = 0.0,
			double initialValueV = 0.0, double initialValueW = 0.0);
		FaceCenteredGrid3(const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing = Vector3D(1.0, 1.0, 1.0),
			const Vector3D& origin = Vector3D::zero,
			const Vector3D& initialValue = Vector3D::zero);
		double& u(size_t i, size_t j, size_t k);
		const double& u(size_t i, size_t j, size_t k) const;
		double& v(size_t i, size_t j, size_t k);
		const double& v(size_t i, size_t j, size_t k) const;
		double& w(size_t i, size_t j, size_t k);
		const double& w(size_t i, size_t j, size_t k) const;
	protected:
		void onResize(const Vector3<size_t>& resolution, const Vector3D& gridSpacing,
			const Vector3D& origin, const Vector3D& initialValue) final;
	private:
		Array3<double> _dataU;
		Array3<double> _dataV;
		Array3<double> _dataW;
		Vector3D _dataOriginU;
		Vector3D _dataOriginV;
		Vector3D _dataOriginW;
	};

	class CollocatedVectorGrid3 : public VectorGrid3 {
	public:
		CollocatedVectorGrid3();
		virtual ~CollocatedVectorGrid3();
		//**********************************************
		//Returns the actual data point size.
		//**********************************************
		virtual Vector3<size_t> GetDataSize() const = 0;

		//**********************************************
		//Returns the origin of the grid data.
		//**********************************************
		virtual Vector3D GetDataOrigin() const = 0;

		const Vector3D& operator()(size_t i, size_t j, size_t k) const;
		Vector3D& operator()(size_t i, size_t j, size_t k);
	protected:
		void onResize(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing,
			const Vector3D& origin,
			const Vector3D& initialValue) final;
	private:
		Array3<Vector3D> _data;
	};

	class CellCenteredVectorGrid3 final : public CollocatedVectorGrid3 {
	public:
		CellCenteredVectorGrid3();
		CellCenteredVectorGrid3(
			size_t resolutionX,
			size_t resolutionY,
			size_t resolutionZ,
			double gridSpacingX = 1.0,
			double gridSpacingY = 1.0,
			double gridSpacingZ = 1.0,
			double originX = 0.0,
			double originY = 0.0,
			double originZ = 0.0,
			double initialValueU = 0.0,
			double initialValueV = 0.0,
			double initialValueW = 0.0);
		CellCenteredVectorGrid3(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing = Vector3D(1.0, 1.0, 1.0),
			const Vector3D& origin = Vector3D::zero,
			const Vector3D& initialValue = Vector3D::zero);
		virtual Vector3<size_t> GetDataSize() const override;
		virtual Vector3D GetDataOrigin() const override;
	};

	class VertexCenteredVectorGrid3 final : public CollocatedVectorGrid3 {
	public:
		VertexCenteredVectorGrid3();
		VertexCenteredVectorGrid3(
			size_t resolutionX,
			size_t resolutionY,
			size_t resolutionZ,
			double gridSpacingX = 1.0,
			double gridSpacingY = 1.0,
			double gridSpacingZ = 1.0,
			double originX = 0.0,
			double originY = 0.0,
			double originZ = 0.0,
			double initialValueU = 0.0,
			double initialValueV = 0.0,
			double initialValueW = 0.0);
		VertexCenteredVectorGrid3(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing = Vector3D(1.0, 1.0, 1.0),
			const Vector3D& origin = Vector3D::zero,
			const Vector3D& initialValue = Vector3D::zero);
		virtual Vector3<size_t> GetDataSize() const override;
		virtual Vector3D GetDataOrigin() const override;
	};

	class ScalarGridBuilder3 {
	public:
		ScalarGridBuilder3();
		virtual ~ScalarGridBuilder3();

		//**********************************************
		//Returns 3-D scalar grid with given parameters.
		//**********************************************
		virtual std::shared_ptr<ScalarGrid3> Build(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing,
			const Vector3D& gridOrigin,
			double initialVal) const = 0;
	};

	class VectorGridBuilder3 {
	public:
		VectorGridBuilder3();
		virtual ~VectorGridBuilder3();

		//**********************************************
		//Returns 3-D Vector grid with given parameters.
		//**********************************************
		virtual std::shared_ptr<VectorGrid3> Build(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing,
			const Vector3D& gridOrigin,
			const Vector3D& initialVal) const = 0;
	};
}
#endif
