#ifndef _OutputForMatplotlib_
#define _OutputForMatplotlib_

#include <pystring\pystring.h>
#include <direct.h>
#include <cnpy\cnpy.h>

inline void createDirectory(const std::string& dirname) {
	std::vector<std::string> tokens;
	pystring::split(dirname, tokens, "/");
	std::string partialDir;
	for (const auto& token : tokens) {
		partialDir = pystring::os::path::join(partialDir, token);
		_mkdir(partialDir.c_str());
	}
}

#endif