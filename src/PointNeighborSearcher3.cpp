#include "PointNeighborSearcher3.h"

using namespace CalfFluidEngine;

PointNeighborSearcher3::PointNeighborSearcher3()
{
}


PointNeighborSearcher3::~PointNeighborSearcher3()
{
}

CalfFluidEngine::PointHashGridSearcher3::PointHashGridSearcher3(const Vector3<size_t>& resolution, double gridSpacing)
	:
	_gridSpacing(gridSpacing), _resolution(resolution)
{
}

void CalfFluidEngine::PointHashGridSearcher3::Build(const std::vector<Vector3D>& points)
{
	_buckets.clear();
	_points.clear();

	size_t points_size = points.size();
	if (points_size == 0) {
		return;
	}

	_buckets.resize(_resolution.x * _resolution.y * _resolution.z);
	_points.resize(points_size);

	for (size_t i = 0; i < points_size; ++i) {
		_points[i] = points[i];
		size_t key = GetHashKeyFromPosition(points[i]);
		_buckets[key].push_back(i);
	}
}

void CalfFluidEngine::PointHashGridSearcher3::ForEachNearbyPoint(const Vector3D & origin, double radius, const std::function<void(size_t, const Vector3D&)>& callback) const
{
	if (_buckets.empty()) {
		return;
	}

	size_t nearbyKeys[8];
	getNearbyKeys(origin, nearbyKeys);

	const double queryRadiusSquared = radius * radius;

	for (int i = 0; i < 8; i++) {
		const auto& bucket = _buckets[nearbyKeys[i]];
		size_t numberOfPointsInBucket = bucket.size();

		for (size_t j = 0; j < numberOfPointsInBucket; ++j) {
			size_t pointIndex = bucket[j];
			double rSquared = (_points[pointIndex] - origin).SquareMagnitude();
			if (rSquared <= queryRadiusSquared) {
				callback(pointIndex, _points[pointIndex]);
			}
		}
	}
}

size_t CalfFluidEngine::PointHashGridSearcher3::GetHashKeyFromPosition(const Vector3D & position) const
{
	Vector3<size_t> bucketIndex = GetBucketIndex(position);
	return GetHashKeyFromBucketIndex(bucketIndex);
}

size_t CalfFluidEngine::PointHashGridSearcher3::GetHashKeyFromBucketIndex(const Vector3<size_t>& bucketIndex) const {
	Vector3<size_t> wrappedIndex = bucketIndex;
	wrappedIndex.x = bucketIndex.x / _resolution.x;
	wrappedIndex.y = bucketIndex.y / _resolution.y;
	wrappedIndex.z = bucketIndex.z / _resolution.z;
	wrappedIndex.x = bucketIndex.x - _resolution.x * wrappedIndex.x;
	wrappedIndex.y = bucketIndex.y - _resolution.y * wrappedIndex.y;
	wrappedIndex.z = bucketIndex.z - _resolution.z * wrappedIndex.z;
	if (wrappedIndex.x < 0) {
		wrappedIndex.x += _resolution.x;
	}
	if (wrappedIndex.y < 0) {
		wrappedIndex.y += _resolution.y;
	}
	if (wrappedIndex.z < 0) {
		wrappedIndex.z += _resolution.z;
	}
	return (wrappedIndex.z 
		* _resolution.y + wrappedIndex.y) 
		* _resolution.x + wrappedIndex.x;
}

Vector3<size_t> CalfFluidEngine::PointHashGridSearcher3::GetBucketIndex(const Vector3D & position) const
{
	Vector3<size_t> bucketIndex;
	bucketIndex.x = static_cast<size_t>(
		std::floor(position.x / _gridSpacing));
	bucketIndex.y = static_cast<size_t>(
		std::floor(position.y / _gridSpacing));
	bucketIndex.z = static_cast<size_t>(
		std::floor(position.z / _gridSpacing));
	return bucketIndex;
}

void CalfFluidEngine::PointHashGridSearcher3::getNearbyKeys(const Vector3D & position, size_t * nearbyKeys) const
{
	Vector3<size_t> originIndex = GetBucketIndex(position), nearbyBucketIndices[8];

	for (int i = 0; i < 8; i++) {
		nearbyBucketIndices[i] = originIndex;
	}

	if ((originIndex.x + 0.5f) * _gridSpacing <= position.x) {
		nearbyBucketIndices[4].x += 1;
		nearbyBucketIndices[5].x += 1;
		nearbyBucketIndices[6].x += 1;
		nearbyBucketIndices[7].x += 1;
	}
	else {
		nearbyBucketIndices[4].x -= 1;
		nearbyBucketIndices[5].x -= 1;
		nearbyBucketIndices[6].x -= 1;
		nearbyBucketIndices[7].x -= 1;
	}

	if ((originIndex.y + 0.5f) * _gridSpacing <= position.y) {
		nearbyBucketIndices[2].y += 1;
		nearbyBucketIndices[3].y += 1;
		nearbyBucketIndices[6].y += 1;
		nearbyBucketIndices[7].y += 1;
	}
	else {
		nearbyBucketIndices[2].y -= 1;
		nearbyBucketIndices[3].y -= 1;
		nearbyBucketIndices[6].y -= 1;
		nearbyBucketIndices[7].y -= 1;
	}

	if ((originIndex.z + 0.5f) * _gridSpacing <= position.z) {
		nearbyBucketIndices[1].z += 1;
		nearbyBucketIndices[3].z += 1;
		nearbyBucketIndices[5].z += 1;
		nearbyBucketIndices[7].z += 1;
	}
	else {
		nearbyBucketIndices[1].z -= 1;
		nearbyBucketIndices[3].z -= 1;
		nearbyBucketIndices[5].z -= 1;
		nearbyBucketIndices[7].z -= 1;
	}

	for (int i = 0; i < 8; i++) {
		nearbyKeys[i] = GetHashKeyFromBucketIndex(nearbyBucketIndices[i]);
	}

}
