/*
 * controller_ui.cpp
 *
 *  Created on: 2021. 3. 3.
 *      Author: RNB_CAD
 */

#include "controller_ui.h"

#ifdef _WIN32
#include <windows.h>
#include <tchar.h>
int RNB::list_dir(std::string dir, std::vector<std::string> &files,
	bool remove_extension) {

	char path[MAX_PATH];
	WIN32_FIND_DATA ffd;

	if (dir.size() > MAX_PATH - 4)
	{
		std::cerr << _T("Your path is too long.") << std::endl;
		return -1;
	}

	_tcsncpy(path, dir.c_str(), MAX_PATH);
	if (path[_tcslen(path) - 1] != '\\')
		_tcscat(path, _T("\\"));
	_tcscat(path, _T("*.*"));

	HANDLE hFind = FindFirstFile(path, &ffd);
	if (hFind == INVALID_HANDLE_VALUE)
	{
		std::cerr << _T("Invalid handle value.") << GetLastError() << std::endl;
		return -1;
	}

	files.clear();
	std::string filename;
	bool finished = false;
	while (!finished)
	{
		filename = ffd.cFileName;
		if (!FindNextFile(hFind, &ffd)) {
			finished = true;
		}

		if (filename == "." || filename == "..") {
			continue;
		}

		if (remove_extension) {
			int idx_ext = filename.rfind(".");
			if (idx_ext > 0) {
				filename = filename.substr(0, idx_ext);
			}
		}

		files.push_back(filename);
	}

	return 0;
}
#else
int RNB::list_dir(std::string dir, std::vector<std::string> &files,
		bool remove_extension) {
	DIR *dp;
	struct dirent *dirp;
	if ((dp = opendir(dir.c_str())) == NULL) {
		std::cout << "Error(" << errno << ") opening " << dir << std::endl;
		return errno;
	}

	files.clear();
	while ((dirp = readdir(dp)) != NULL) {
		std::string filename = std::string(dirp->d_name);
		if (filename == "." || filename == "..") {
			continue;
		}
		if (remove_extension) {
			int idx_ext = filename.rfind(".");
			if (idx_ext > 0) {
				filename = filename.substr(0, idx_ext);
			}
		}
		files.push_back(filename);
	}
	closedir(dp);
	return 0;
}
#endif

void RNB::update_controller_list(Dropdown &dropdown_controller, const char* lib_path) {
	std::vector<std::string> controller_names; /**< controller name list */

    mkdir(lib_path, 0755);
	list_dir(lib_path, controller_names, true);
	dropdown_controller.clear_items();
	for (auto itor = controller_names.begin(); itor != controller_names.end();
			itor++) {
		dropdown_controller.add_item(itor->c_str(), itor->c_str());
	}
}
