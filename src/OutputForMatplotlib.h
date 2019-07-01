#ifndef _OutputForMatplotlib_
#define _OutputForMatplotlib_
 
#include <direct.h>
#include <vector>
#include <Array3.h>
#include "../External/cnpy/cnpy.h"
#include "../External/pystring/pystring.h"
#include <ParticleSystemSolver3.h>
class OutputForMatplotlib final{
public:
	OutputForMatplotlib(const std::string& testDemoName){
		_testCollectionDir = "..\\Output";
		createDirectory(_testCollectionDir);
		createTestDirectory(testDemoName);
		
	}

	template<typename T>
	void SaveData(
		const std::vector<T>& data, 
		const std::string& name) {
		std::string filename = getFullFilePath(name); 
		unsigned int dim[1] = { 
			static_cast<unsigned int>(data.size()) 
		}; 
		cnpy::npy_save(filename, data.data(), dim, 1, "w"); 
	}

	template<typename T>
	void SaveData(
		const std::vector<T>& data,
		size_t size, const std::string& name){
		std::string filename = getFullFilePath(name);
		unsigned int dim[1] = {
			static_cast<unsigned int>(size)
		};
		cnpy::npy_save(filename, data.data(), dim, 1, "w");
	}

	template<typename T>
	void SaveData(
		const std::vector<T>& data,
		size_t sizeX,size_t sizeY, const std::string& name) {
		std::string filename = getFullFilePath(name);
		unsigned int dim[2] = {
			static_cast<unsigned int>(sizeY),
			static_cast<unsigned int>(sizeX)
		};
		cnpy::npy_save(filename, data.data(), dim, 2, "w");
	}

	template <typename T> 
	void SaveData(
		const Array3<T>& data, 
		const std::string& name) 
	{
		std::string filename = getFullFilePath(name); 
		unsigned int dim[3] = { 
		static_cast<unsigned int>(data.Size().z),
		static_cast<unsigned int>(data.Size().y),
		static_cast<unsigned int>(data.Size().x)
		}; 
		cnpy::npy_save(filename, data.data(), dim, 3, "w"); 
	} 

	template<typename ParticleSystem>
	void SaveParticleDataXY(
		const std::shared_ptr<ParticleSystem>& particles,
		unsigned int frameNum) {
			size_t n = particles->GetNumberOfParticles();
			std::vector<double> x(n); 
			std::vector<double> y(n);
			auto positions = particles->GetPositions(); 
			for (size_t i = 0; i < n; ++i) {
					x[i] = positions[i].x;
					y[i] = positions[i].y;
			}
			char filename[256]; 
			snprintf(
				filename, 
				sizeof(filename), 
				"data.#point2,%04d,x.npy", 
				frameNum); 
			SaveData(x, filename);
			snprintf(
				filename, 
				sizeof(filename),
				"data.#point2,%04d,y.npy", 
				frameNum); 
			SaveData(y, filename);
	}
private:
	inline void createDirectory(const std::string& dirname) {
		std::vector<std::string> tokens;
		pystring::split(dirname, tokens, "\\");
		std::string partialDir;
		for (const auto& token : tokens) {
			partialDir = pystring::os::path::join(partialDir, token);
			_mkdir(partialDir.c_str());
		}
	}

	std::string getFullFilePath(const std::string& name) {
		if (!_currentTestDir.empty()) {
			return pystring::os::path::join(_currentTestDir, name); 
		}
		else {
			return name;
		}
	}

	std::string getTestDirectoryName(const std::string& name) {
		return pystring::os::path::join(_testCollectionDir, name);
	}

	void createTestDirectory(const std::string& name) {
		_currentTestDir = getTestDirectoryName(name);
		createDirectory(_currentTestDir); 
	} 

	std::string _testCollectionDir; 
	std::string _currentTestCaseName; 
	std::string _currentTestDir; 
};

#endif