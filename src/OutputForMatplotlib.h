#ifndef _OutputForMatplotlib_
#define _OutputForMatplotlib_
 
#include <direct.h>
#include <vector>

#include "../External/cnpy/cnpy.h"
#include "../External/pystring/pystring.h"

class OutputForMatplotlib final
{
public:
	OutputForMatplotlib(const std::string& testDemoName)
	{
		_testCollectionDir = "..\\Output";
		createDirectory(_testCollectionDir);
		createTestDirectory(testDemoName);
		
	}

	template<typename T>
	void SaveData(
		const const std::vector<T>& data, 
		const std::string& name) 
	{
		std::string filename = getFullFilePath(name); 
		unsigned int dim[1] = { 
			static_cast<unsigned int>(data.size()) 
		}; 
		cnpy::npy_save(filename, data.data(), dim, 1, "w"); 
	}

	template<typename T>
	void SaveData(
		const const std::vector<T>& data,
		size_t size, const std::string& name)
	{
		std::string filename = getFullFilePath(name);
		unsigned int dim[1] = {
			static_cast<unsigned int>(size)
		};
		cnpy::npy_save(filename, data.data(), dim, 1, "w");
	}
private:
	inline void createDirectory(const std::string& dirname) 
	{
		std::vector<std::string> tokens;
		pystring::split(dirname, tokens, "\\");
		std::string partialDir;
		for (const auto& token : tokens) {
			partialDir = pystring::os::path::join(partialDir, token);
			_mkdir(partialDir.c_str());
		}
	}

	std::string getFullFilePath(const std::string& name) 
	{
		if (!_currentTestDir.empty()) {
			return pystring::os::path::join(_currentTestDir, name); 
		}
		else {
			return name;
		}
	}

	std::string getTestDirectoryName(const std::string& name) 
	{
		return pystring::os::path::join(_testCollectionDir, name);
	}

	void createTestDirectory(const std::string& name) 
	{
		_currentTestDir = getTestDirectoryName(name);
		createDirectory(_currentTestDir); 
	} 

	std::string _testCollectionDir; 
	std::string _currentTestCaseName; 
	std::string _currentTestDir; 
};

#endif