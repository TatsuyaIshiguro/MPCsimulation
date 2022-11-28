#pragma once
#include <string>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

class SaveParam {
public:

	static constexpr char* parameterPath = (char*)"C:\\MPCLauncher\\MPCLauncher\\Parameter_setting\\parameter.csv";

	void save_prm(std::string save_path) {
		fs::path nuopt_prm_path = parameterPath;
		fs::path to_path_prm = save_path;
		fs::copy_file(nuopt_prm_path, to_path_prm);
		
	}
};

