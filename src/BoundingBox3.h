#ifndef _CalfFluidEngine_BoundingBox3_
#define _CalfFluidEngine_BoundingBox3_
#include <Vector3.h>
#include <Ray3.h>
#include <Math_utils.h>
namespace CalfFluidEngine {
	template <typename T>
	struct BoundingBoxRayIntersection3 {
		//! True if the box and ray intersects.
		bool isIntersecting = false;

		//! Distance to the first intersection point.
		T tNear = std::numeric_limits<T>::max();

		//! Distance to the second (and the last) intersection point.
		T tFar = std::numeric_limits<T>::max();
	};

	template <typename T>
	class BoundingBox3 {
	public:
		Vector3<T> lowerCorner;
		Vector3<T> upperCorner;
		BoundingBox3() { Reset(); }
		BoundingBox3(const Vector3<T>& point1, const Vector3<T>& point2)
		{
			lowerCorner.x = std::min(point1.x, point2.x);
			lowerCorner.y = std::min(point1.y, point2.y);
			lowerCorner.z = std::min(point1.z, point2.z);
			upperCorner.x = std::max(point1.x, point2.x);
			upperCorner.y = std::max(point1.y, point2.y);
			upperCorner.z = std::max(point1.z, point2.z);
		}
		T GetWidth() const{ return upperCorner.x - lowerCorner.x; }
		T GetHeight() const{ return upperCorner.y - lowerCorner.y; }
		T GetDepth() const { return upperCorner.z - lowerCorner.z; }
		T GetLength(size_t axis) const{ return upperCorner[axis] - lowerCorner[axis]; }

		//! Returns true of this box and other box overlaps.
		bool Overlaps(const BoundingBox3<T>& other) const
		{
			if (upperCorner.x < other.lowerCorner.x ||
				lowerCorner.x > other.upperCorner.x) {
				return false;
			}

			if (upperCorner.y < other.lowerCorner.y ||
				lowerCorner.y > other.upperCorner.y) {
				return false;
			}

			if (upperCorner.z < other.lowerCorner.z ||
				lowerCorner.z > other.upperCorner.z) {
				return false;
			}

			return true;
		}

		//! Returns true if the input point is inside of this box.
		bool Contains(const Vector3<T>& point) const
		{
			if (upperCorner.x < point.x || lowerCorner.x > point.x) {
				return false;
			}

			if (upperCorner.y < point.y || lowerCorner.y > point.y) {
				return false;
			}

			if (upperCorner.z < point.z || lowerCorner.z > point.z) {
				return false;
			}

			return true;
		}

		//! Returns true if the input ray is intersecting with this box.
		bool Intersects(const Ray3<T>& ray) const
		{
			T tMin = 0;
			T tMax = std::numeric_limits<T>::max();
			const Vector3<T>& rayInvDir = Vector3<T>(1,1,1) /ray.direction;

			for (int i = 0; i < 3; ++i) {
				T tNear = (lowerCorner[i] - ray.origin[i]) * rayInvDir[i];
				T tFar = (upperCorner[i] - ray.origin[i]) * rayInvDir[i];

				if (tNear > tFar) std::swap(tNear, tFar);
				tMin = tNear > tMin ? tNear : tMin;
				tMax = tFar < tMax ? tFar : tMax;

				if (tMin > tMax) return false;
			}

			return true;
		}

		//! Returns intersection.isIntersecting = true if the input ray is
		//! intersecting with this box. If interesects, intersection.tNear is
		//! assigned with distant to the closest intersecting point, and
		//! intersection.tFar with furthest.
		BoundingBoxRayIntersection3<T> GetClosestIntersection(
			const Ray3<T>& ray) const
		{
			BoundingBoxRayIntersection3<T> intersection;

			T tMin = 0;
			T tMax = std::numeric_limits<T>::max();
			const Vector3<T>& rayInvDir = Vector3<T>(1,1,1) / ray.direction;

			for (int i = 0; i < 3; ++i) {
				T tNear = (lowerCorner[i] - ray.origin[i]) * rayInvDir[i];
				T tFar = (upperCorner[i] - ray.origin[i]) * rayInvDir[i];

				if (tNear > tFar) std::swap(tNear, tFar);
				tMin = tNear > tMin ? tNear : tMin;
				tMax = tFar < tMax ? tFar : tMax;

				if (tMin > tMax) {
					intersection.isIntersecting = false;
					return intersection;
				}
			}

			intersection.isIntersecting = true;

			if (contains(ray.origin)) {
				intersection.tNear = tMax;
				intersection.tFar = std::numeric_limits<T>::max();
			}
			else {
				intersection.tNear = tMin;
				intersection.tFar = tMax;
			}

			return intersection;
		}

		Vector3<T> GetMidPoint() const{ return (upperCorner + lowerCorner) / static_cast<T>(2); }
		T GetDiagonalLength() const{ return (upperCorner - lowerCorner).Magnitude(); }
		T GetDiagonalLengthSquared() const{ return (upperCorner - lowerCorner).SquareMagnitude(); }

		void Reset()
		{
			lowerCorner.x = std::numeric_limits<T>::max();
			lowerCorner.y = std::numeric_limits<T>::max();
			lowerCorner.z = std::numeric_limits<T>::max();
			upperCorner.x = -std::numeric_limits<T>::max();
			upperCorner.y = -std::numeric_limits<T>::max();
			upperCorner.z = -std::numeric_limits<T>::max();
		}
		void Merge(const Vector3<T>& point)
		{
			lowerCorner.x = std::min(lowerCorner.x, point.x);
			lowerCorner.y = std::min(lowerCorner.y, point.y);
			lowerCorner.z = std::min(lowerCorner.z, point.z);
			upperCorner.x = std::max(upperCorner.x, point.x);
			upperCorner.y = std::max(upperCorner.y, point.y);
			upperCorner.z = std::max(upperCorner.z, point.z);
		}
		void Merge(const BoundingBox3<T>& other)
		{
			lowerCorner.x = std::min(lowerCorner.x, other.lowerCorner.x);
			lowerCorner.y = std::min(lowerCorner.y, other.lowerCorner.y);
			lowerCorner.z = std::min(lowerCorner.z, other.lowerCorner.z);
			upperCorner.x = std::max(upperCorner.x, other.upperCorner.x);
			upperCorner.y = std::max(upperCorner.y, other.upperCorner.y);
			upperCorner.z = std::max(upperCorner.z, other.upperCorner.z);
		}

		//! Expands this box by given delta to all direction.
		//! If the width of the box was x, expand(y) will result a box with
		//! x+y+y width.
		void Expand(T delta)
		{
			for (size_t i = 0; i < 3; ++i)
			{
				lowerCorner[i] -= delta;
				upperCorner[i] += delta;
			}
		}

		//! Returns corner position. Index starts from x-first order.
		Vector3<T> GetCorner(size_t idx) const
		{
			static const T h = static_cast<T>(1) / 2;
			static const Vector3<T> offset[8] = {
				{ -h, -h, -h },{ +h, -h, -h },{ -h, +h, -h },{ +h, +h, -h },
			{ -h, -h, +h },{ +h, -h, +h },{ -h, +h, +h },{ +h, +h, +h } };

			return Vector3<T>(GetWidth(), GetHeight(), GetDepth()) * offset[idx] + GetMidPoint();
		}

		//! Returns the clamped point.
		Vector3<T> Clamp(const Vector3<T>& pt) const
		{
			return Clamp(pt, lowerCorner, upperCorner);
		}

	};

	typedef BoundingBox3<float> BoundingBox3F;
	typedef BoundingBox3<double> BoundingBox3D;
}
#endif