#ifndef _CalfFluidEngine_PointNeighborSearcher3_
#define _CalfFluidEngine_PointNeighborSearcher3_
#include <vector>
#include <Vector3.h>
#include <functional>
#include <algorithm>

namespace CalfFluidEngine {
	class PointNeighborSearcher3
	{
	public:
		PointNeighborSearcher3();
		~PointNeighborSearcher3();

		//**********************************************
		//building the internal data structure
		//**********************************************
		virtual void Build(const std::vector<Vector3D>& points) = 0;

		//**********************************************
		//looking up the nearby points.
		//**********************************************
		virtual void ForEachNearbyPoint(
			const Vector3D& origin,
			double radius,
			const std::function<void(size_t, const Vector3D&)>& callback) const = 0;

		//**********************************************
		//Returns true if there are any nearby points for given origin within radius
		//**********************************************
		virtual bool HasNearbyPoint(
			const Vector3D& origin, double radius) const = 0;
	};

	class PointHashGridSearcher3 final : public PointNeighborSearcher3
	{
	public:
		PointHashGridSearcher3(const Vector3<size_t>& resolution, double gridSpacing);
		virtual void Build(const std::vector<Vector3D>& points);
		virtual void ForEachNearbyPoint(
			const Vector3D& origin,
			double radius,
			const std::function<void(size_t, const Vector3D&)>& callback) const;
		size_t GetHashKeyFromPosition(const Vector3D& position) const;
		size_t GetHashKeyFromBucketIndex(const Vector3<size_t>& bucketIndex) const;
		Vector3<size_t> GetBucketIndex(const Vector3D& position) const;
		bool HasNearbyPoint(
			const Vector3D& origin, double radius) const override;
		void Add(const Vector3D& point);
	private:
		void getNearbyKeys(const Vector3D& position, size_t* nearbyKeys) const;
		double _gridSpacing = 1.0;
		std::vector<Vector3D> _points;
		std::vector<std::vector<size_t>> _buckets;
		Vector3<size_t> _resolution = Vector3<size_t>(1, 1, 1);
	};
}
#endif
