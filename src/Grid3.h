#ifndef _CalfFluidEngine_Grid3_
#define _CalfFluidEngine_Grid3_
#include <Vector3.h>
#include <BoundingBox3.h>
#include <Field3.h>
#include <Array3.h>
#include <memory>
#include <ArraySampler.h>
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
		std::function<Vector3D(size_t, size_t, size_t)> GetCellCenterPosition() const;
		void SetGrid(const Grid3& other);

		//**********************************************
		//Swaps the data with other grid.
		//**********************************************
		virtual void Swap(Grid3* other);
	protected:
		void setSizeParameters(
			const Vector3<size_t>& resolution, 
			const Vector3D& gridSpacing,
			const Vector3D& origin);
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
		const double& operator()(size_t i, size_t j, size_t k) const;
		double& operator()(size_t i, size_t j, size_t k);

		//**********************************************
		//Returns the size of the grid data.
		//equal to the grid resolution if the data is not stored at cell-center.
		//**********************************************
		virtual Vector3<size_t> GetDataSize() const = 0;

		//**********************************************
		//Returns the origin of the grid data.
		//**********************************************
		virtual Vector3D GetDataOrigin() const = 0;

		//**********************************************
		//Returns the copy of the grid instance.
		//**********************************************
		virtual std::shared_ptr<ScalarGrid3> Clone() const = 0;

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

		Vector3D GetGradientAtDataPoint(size_t i, size_t j, size_t k) const;
		double GetLaplacianAtDataPoint(size_t i, size_t j, size_t k) const;
		virtual Vector3D Gradient(const Vector3D& x) const override;
		virtual double Laplacian(const Vector3D& x) const override;
		virtual double Sample(const Vector3D& x) const override;
		void ParallelForEach(const std::function<void(size_t, size_t, size_t)>& func) const;

		Array3<double>& GetArray3Data();
		const Array3<double>& GetArray3Data() const;
		std::function<Vector3D(size_t, size_t, size_t)> Position() const;

		void Fill(double value);
		void Fill(const std::function<double(const Vector3D&)>& func);

		virtual void Swap(Grid3* other) override;
	private:
		void resetSampler();
		LinearArraySampler3<double, double> _linearSampler;
		std::function<double(const Vector3D&)> _sampler;
		Array3<double> _data;
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

		//**********************************************
		//Returns the copy of the grid instance.
		//**********************************************
		virtual std::shared_ptr<VectorGrid3> Clone() const = 0;

		//**********************************************
		//Fills the grid with given value.
		//**********************************************
		virtual void Fill(Vector3D value) = 0;

		//**********************************************
		//Fills the grid with given position-to-value mapping function.
		//**********************************************
		virtual void Fill(const std::function<Vector3D(const Vector3D&)>& func) = 0;
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
		std::shared_ptr<ScalarGrid3> Clone() const override;
		//void Swap(Grid3* other) override;
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
		std::shared_ptr<ScalarGrid3> Clone() const override;
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
		FaceCenteredGrid3(const FaceCenteredGrid3& other);
		double& u(size_t i, size_t j, size_t k);
		const double& u(size_t i, size_t j, size_t k) const;
		double& v(size_t i, size_t j, size_t k);
		const double& v(size_t i, size_t j, size_t k) const;
		double& w(size_t i, size_t j, size_t k);
		const double& w(size_t i, size_t j, size_t k) const;
		Array3<double>& UArray3();
		const Array3<double>& UArray3() const;
		Array3<double>& VArray3();
		const Array3<double>& VArray3() const;
		Array3<double>& WArray3();
		const Array3<double>& WArray3() const;
		std::function<Vector3D(size_t, size_t, size_t)> UPosition() const;
		std::function<Vector3D(size_t, size_t, size_t)> VPosition() const;
		std::function<Vector3D(size_t, size_t, size_t)> WPosition() const;
		double GetDivergenceAtCellCenter(size_t i, size_t j, size_t k) const;
		Vector3D GetCurlAtCellCenter(size_t i, size_t j, size_t k) const;
		Vector3D GetValueAtCellCenter(size_t i, size_t j, size_t k) const;
		virtual double Divergence(const Vector3D& x) const override;
		virtual Vector3D Curl(const Vector3D& x) const override;
		std::function<Vector3D(const Vector3D&)> Sampler() const override;
		Vector3D Sample(const Vector3D& x) const override;
		virtual std::shared_ptr<VectorGrid3> Clone() const override;
		virtual void Fill(Vector3D value) override;
		virtual void Fill(const std::function<Vector3D(const Vector3D&)>& func) override;
		void Swap(Grid3* other) override;
	protected:
		void onResize(const Vector3<size_t>& resolution, const Vector3D& gridSpacing,
			const Vector3D& origin, const Vector3D& initialValue) final;
	private:
		void resetSampler();
		Array3<double> _dataU;
		Array3<double> _dataV;
		Array3<double> _dataW;
		Vector3D _dataOriginU;
		Vector3D _dataOriginV;
		Vector3D _dataOriginW;
		LinearArraySampler3<double, double> _uLinearSampler;
		LinearArraySampler3<double, double> _vLinearSampler;
		LinearArraySampler3<double, double> _wLinearSampler;
		std::function<Vector3D(const Vector3D&)> _sampler;
	};

	class CollocatedVectorGrid3 : public VectorGrid3 {
	public:
		CollocatedVectorGrid3();
		virtual ~CollocatedVectorGrid3();
		const Vector3D& operator()(size_t i, size_t j, size_t k) const;
		Vector3D& operator()(size_t i, size_t j, size_t k);
		double GetDivergenceAtDataPoint(size_t i, size_t j, size_t k) const;
		Vector3D GetCurlAtDataPoint(size_t i, size_t j, size_t k) const;

		//**********************************************
		//Returns the actual data point size.
		//**********************************************
		virtual Vector3<size_t> GetDataSize() const = 0;

		//**********************************************
		//Returns the origin of the grid data.
		//**********************************************
		virtual Vector3D GetDataOrigin() const = 0;

		virtual double Divergence(const Vector3D& x) const override;
		virtual Vector3D Curl(const Vector3D& x) const override;
		std::function<Vector3D(const Vector3D&)> Sampler() const override;
		virtual Vector3D Sample(const Vector3D& x) const override;
		void Swap(Grid3* other) override;
		virtual void Fill(Vector3D value) override;
		virtual void Fill(const std::function<Vector3D(const Vector3D&)>& func) override;
		std::function<Vector3D(size_t, size_t, size_t)> Position() const;

		void ParallelForEach(const std::function<void(size_t, size_t, size_t)>& func) const;

		Array3<Vector3D>& GetArray3Data();
		const Array3<Vector3D>& GetArray3Data() const;
	protected:
		void onResize(
			const Vector3<size_t>& resolution,
			const Vector3D& gridSpacing,
			const Vector3D& origin,
			const Vector3D& initialValue) final;
	private:
		void resetSampler();
		LinearArraySampler3<Vector3D, double> _linearSampler;
		std::function<Vector3D(const Vector3D&)> _sampler;
		Array3<Vector3D> _data = Array3<Vector3D>();
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
		std::shared_ptr<VectorGrid3> Clone() const override;
		
		
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
		std::shared_ptr<VectorGrid3> Clone() const override;
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
