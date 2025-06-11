//
// Created by rnb on 21. 3. 2..
//

#ifndef WEBTEST_CONTROLLER_UI_H
#define WEBTEST_CONTROLLER_UI_H

#include <stdio.h>
#ifdef _WIN32
#else
#include <dirent.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <dlfcn.h>
#endif
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include <errno.h>
#include <iostream>
#include <memory>
#include "html_contents.h"
#include "controller_interface.h"

namespace RNB {

/**
 * @brief list files in a directory
 * @param dir path name
 * @param files string vector to return file names
 * @param remove_extension to remove file extensions in the file names
 * @return
 */
int list_dir(std::string dir, std::vector<std::string> &files,
		bool remove_extension = false);


/**
 * @brief upate controller list from directory
 */
void update_controller_list(Dropdown &dropdown_controller, const char* lib_path);

}
#endif //WEBTEST_CONTROLLER_UI_H
