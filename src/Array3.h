#ifndef _CalfFluidEngine_Array3_
#define _CalfFluidEngine_Array3_
#include <Vector3.h>
#include <vector>
namespace CalfFluidEngine {
	template <typename T>
	class Array3 final
	{
	public:
		Array3() {};
		void Resize(const Vector3<size_t>& size, const T& initVal = T())
		{
			Array3 grid;
			grid._data.resize(size.x * size.y * size.z, initVal);
			grid._size = size;
			size_t iMin = std::min(size.x, _size.x);
			size_t jMin = std::min(size.y, _size.y);
			size_t kMin = std::min(size.z, _size.z);
			for (size_t k = 0; k < kMin; ++k) {
				for (size_t j = 0; j < jMin; ++j) {
					for (size_t i = 0; i < iMin; ++i) {
						grid(i, j, k) = at(i, j, k);
					}
				}
			}

			Swap(grid);
		}
		T& at(size_t i, size_t j, size_t k) {
			return _data[i + _size.x * (j + _size.y * k)];
		}
		const T& at(size_t i, size_t j, size_t k) const {
			return _data[i + _size.x * (j + _size.y * k)];
		}
		T& operator()(size_t i, size_t j, size_t k)
		{
			return _data[i + _size.x * (j + _size.y * k)];
		}
		const T& operator()(size_t i, size_t j, size_t k) const
		{
			return _data[i + _size.x * (j + _size.y * k)];
		}
		T& operator[](size_t i) {
			return _data[i];
		}
		const T& operator[](size_t i) const {
			return _data[i];
		}
		void Swap(Array3& other)
		{
			std::swap(other._data, _data);
			std::swap(other._size, _size);
		}
	private:
		Vector3<size_t> _size;
		std::vector<T> _data;
	};
}
#endif