#ifndef _CalfFluidEngine_BVH_
#define _CalfFluidEngine_BVH_
#include <functional>
#include <vector>
#include <BoundingBox3.h>
#include <Constant.h>
#include <numeric>
namespace CalfFluidEngine {
	template <typename T>
	struct NearestNeighborQueryResult {
		const T* item = nullptr;
		double distance = kMaxD;
	};

	template <typename T>
	struct ClosestIntersectionQueryResult3 {
		const T* item = nullptr;
		double distance = kMaxD;
	};

	template <typename T>
	class BVH3 final
	{
	public:
		BVH3() {}
		virtual ~BVH3() {}

		void Build(const std::vector<T>& items,
			const std::vector<BoundingBox3D>& itemsBounds)
		{
			_items = items;
			_itemBounds = itemsBounds;

			if (_items.empty()) {
				return;
			}

			_nodes.clear();

			for (size_t i = 0; i < _items.size(); ++i) {
				_bound.Merge(_itemBounds[i]);
			}

			std::vector<size_t> itemIndices(_items.size());
			std::iota(std::begin(itemIndices), std::end(itemIndices), 0);

			build(0, itemIndices.data(), _items.size(), 0);
		}

		const BoundingBox3D& GetBoundingBox() const {
			return _bound;
		}

		NearestNeighborQueryResult<T> GetNearest(
			const Vector3D& pt,
			const std::function<double(const T&, const Vector3D&)>& distanceFunc) const
		{
			NearestNeighborQueryResult<T> best;
			best.distance = kMaxD;
			best.item = nullptr;

			// Prepare to traverse BVH
			static const int kMaxTreeDepth = 8 * sizeof(size_t);
			const Node* todo[kMaxTreeDepth];
			size_t todoPos = 0;

			// Traverse BVH nodes
			const Node* node = _nodes.data();
			while (node != nullptr) {
				if (node->IsLeaf()) {
					double dist = distanceFunc(_items[node->item], pt);
					if (dist < best.distance) {
						best.distance = dist;
						best.item = &_items[node->item];
					}

					// Grab next node to process from todo stack
					if (todoPos > 0) {
						// Dequeue
						--todoPos;
						node = todo[todoPos];
					}
					else {
						break;
					}
				}
				else {
					const double bestDistSqr = best.distance * best.distance;

					const Node* left = node + 1;
					const Node* right = &_nodes[node->child];

					// If pt is inside the box, then the closestLeft and Right will be
					// identical to pt. This will make distMinLeftSqr and
					// distMinRightSqr zero, meaning that such a box will have higher
					// priority.
					Vector3D closestLeft = left->bound.Clamp(pt);
					Vector3D closestRight = right->bound.Clamp(pt);

					double distMinLeftSqr = (closestLeft - pt).SquareMagnitude();
					double distMinRightSqr = (closestRight - pt).SquareMagnitude();

					bool shouldVisitLeft = distMinLeftSqr < bestDistSqr;
					bool shouldVisitRight = distMinRightSqr < bestDistSqr;

					const Node* firstChild;
					const Node* secondChild;
					if (shouldVisitLeft && shouldVisitRight) {
						if (distMinLeftSqr < distMinRightSqr) {
							firstChild = left;
							secondChild = right;
						}
						else {
							firstChild = right;
							secondChild = left;
						}

						// Enqueue secondChild in todo stack
						todo[todoPos] = secondChild;
						++todoPos;
						node = firstChild;
					}
					else if (shouldVisitLeft) {
						node = left;
					}
					else if (shouldVisitRight) {
						node = right;
					}
					else {
						if (todoPos > 0) {
							// Dequeue
							--todoPos;
							node = todo[todoPos];
						}
						else {
							break;
						}
					}
				}
			}

			return best;
		}

		ClosestIntersectionQueryResult3<T> GetClosestIntersection(
			const Ray3D& ray, const std::function<double(const T&, const Ray3D&)>& testFunc) const {
			ClosestIntersectionQueryResult3<T> best;
			best.distance = kMaxD;
			best.item = nullptr;

			if (!_bound.Intersects(ray)) {
				return best;
			}

			// prepare to traverse BVH for ray
			static const int kMaxTreeDepth = 8 * sizeof(size_t);
			const Node* todo[kMaxTreeDepth];
			size_t todoPos = 0;

			// traverse BVH nodes for ray
			const Node* node = _nodes.data();

			while (node != nullptr) {
				if (node->IsLeaf()) {
					double dist = testFunc(_items[node->item], ray);
					if (dist < best.distance) {
						best.distance = dist;
						best.item = _items.data() + node->item;
					}

					// grab next node to process from todo stack
					if (todoPos > 0) {
						// Dequeue
						--todoPos;
						node = todo[todoPos];
					}
					else {
						break;
					}
				}
				else {
					// get node children pointers for ray
					const Node* firstChild;
					const Node* secondChild;
					if (ray.direction[node->flags] > 0.0) {
						firstChild = node + 1;
						secondChild = (Node*)&_nodes[node->child];
					}
					else {
						firstChild = (Node*)&_nodes[node->child];
						secondChild = node + 1;
					}

					// advance to next child node, possibly enqueue other child
					if (!firstChild->bound.Intersects(ray)) {
						node = secondChild;
					}
					else if (!secondChild->bound.Intersects(ray)) {
						node = firstChild;
					}
					else {
						// enqueue secondChild in todo stack
						todo[todoPos] = secondChild;
						++todoPos;
						node = firstChild;
					}
				}
			}

			return best;
		}

		bool Intersects(const Ray3D& ray,
			const std::function<bool(const T&, const Ray3D&)>& testFunc) const {
			if (!_bound.Intersects(ray)) {
				return false;
			}

			// prepare to traverse BVH for ray
			static const int kMaxTreeDepth = 8 * sizeof(size_t);
			const Node* todo[kMaxTreeDepth];
			size_t todoPos = 0;

			// traverse BVH nodes for ray
			const Node* node = _nodes.data();

			while (node != nullptr) {
				if (node->IsLeaf()) {
					if (testFunc(_items[node->item], ray)) {
						return true;
					}

					// grab next node to process from todo stack
					if (todoPos > 0) {
						// Dequeue
						--todoPos;
						node = todo[todoPos];
					}
					else {
						break;
					}
				}
				else {
					// get node children pointers for ray
					const Node* firstChild;
					const Node* secondChild;
					if (ray.direction[node->flags] > 0.0) {
						firstChild = node + 1;
						secondChild = (Node*)&_nodes[node->child];
					}
					else {
						firstChild = (Node*)&_nodes[node->child];
						secondChild = node + 1;
					}

					// advance to next child node, possibly enqueue other child
					if (!firstChild->bound.Intersects(ray)) {
						node = secondChild;
					}
					else if (!secondChild->bound.Intersects(ray)) {
						node = firstChild;
					}
					else {
						// enqueue secondChild in todo stack
						todo[todoPos] = secondChild;
						++todoPos;
						node = firstChild;
					}
				}
			}

			return false;
		}

	private:
		struct Node {
			char flags;
			BoundingBox3D bound;
			union {
				size_t child;
				size_t item;
			};


			Node() : flags(0) {
				child = kMaxSize;
			}
			void InitLeaf(size_t it, const BoundingBox3D& b)
			{
				flags = 3;
				item = it;
				bound = b;
			}
			void InitInternal(uint8_t axis, size_t c, const BoundingBox3D& b)
			{
				flags = axis;
				child = c;
				bound = b;
			}
			bool IsLeaf() const {
				return flags == 3;
			}
		};

		BoundingBox3D _bound;
		std::vector<T> _items;
		std::vector<BoundingBox3D> _itemBounds;
		std::vector<Node> _nodes;

		size_t qsplit(size_t* itemIndices, size_t numItems, double pivot,
			uint8_t axis) {
			double centroid;
			size_t ret = 0;
			for (size_t i = 0; i < numItems; ++i) {
				BoundingBox3D b = _itemBounds[itemIndices[i]];
				centroid = 0.5f * (b.lowerCorner[axis] + b.upperCorner[axis]);
				if (centroid < pivot) {
					std::swap(itemIndices[i], itemIndices[ret]);
					ret++;
				}
			}
			if (ret == 0 || ret == numItems) {
				ret = numItems >> 1;
			}
			return ret;
		}

		size_t build(size_t nodeIndex,size_t* itemIndices, size_t nItems,
			size_t currentDepth)
		{
			_nodes.push_back(Node());

			// initialize leaf node if termination criteria met
			if (nItems == 1) {
				_nodes[nodeIndex].InitLeaf(itemIndices[0], _itemBounds[itemIndices[0]]);
				return currentDepth + 1;
			}

			// find the mid-point of the bounding box to use as a qsplit pivot
			BoundingBox3D nodeBound;
			for (size_t i = 0; i < nItems; ++i) {
				nodeBound.Merge(_itemBounds[itemIndices[i]]);
			}

			Vector3D d = nodeBound.upperCorner - nodeBound.lowerCorner;

			// choose which axis to split along
			uint8_t axis;
			if (d.x > d.y && d.x > d.z) {
				axis = 0;
			}
			else {
				axis = (d.y > d.z) ? 1 : 2;
			}

			double pivot =
				0.5 * (nodeBound.upperCorner[axis] + nodeBound.lowerCorner[axis]);

			// classify primitives with respect to split
			size_t midPoint = qsplit(itemIndices, nItems, pivot, axis);

			// recursively initialize children _nodes
			size_t d0 = build(nodeIndex + 1, itemIndices, midPoint, currentDepth + 1);
			_nodes[nodeIndex].InitInternal(axis, _nodes.size(), nodeBound);
			size_t d1 = build(_nodes[nodeIndex].child, itemIndices + midPoint,
				nItems - midPoint, currentDepth + 1);


			return d0 > d1 ? d0 : d1;
		}
	};
}
#endif