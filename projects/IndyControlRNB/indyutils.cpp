/*
 * indyutils.cpp
 *
 *  Created on: 2021. 3. 6.
 *      Author: RNB_CAD
 */

#include "indyutils.h"
#include "encrypt.h"

#include <fstream>
#include <sstream>
#include <vector>

// [NOTE] The developer license was provided under trust from neuromeka. Please do not distribute it or use it for other purposes.
#define ENCRYPT_KEY "RNB-CONTROL"
#define DEVELOPER_LIC_ENCRYPTED "zZl0E0LbFz4Yt7UtrEz2RVh7pWmbOKHZLIgVXtM6wII9SyTwdcPKtU=="
// [NOTE] The developer license was provided under trust from neuromeka. Please do not distribute it or use it for other purposes.

std::string read_license(int idx){
    static bool print_license = true;

	std::string license_info;
	std::ifstream rfile(LICENSE_FILE);
	if(rfile.is_open()){
	    getline(rfile, license_info);
	}
	else{
		std::string lic_enc(DEVELOPER_LIC_ENCRYPTED);
		std::string key(ENCRYPT_KEY);
        license_info = decrypt(lic_enc, key);
        if(print_license){
			print_license = false;
			std::cout<<"==================== [ERROR] SDK License Not Found ====================="<<std::endl;
			std::cout<<"================== Using Developer License For Now... =================="<<std::endl;
			std::cout<<"[WARN] Get SDK license from Neuromeka and enter USERNAME;EMAIL;SERIAL in Components/sdk_license.lic"<<std::endl;
        }
	}
	rfile.close();


    std::vector<std::string> lic_items;
    std::stringstream lstream(license_info);
    std::string temp;

    while (getline(lstream, temp, ';')) {
        lic_items.push_back(temp);
    }

    if(print_license){
        print_license = false;
        std::cout<<"==================== SDK License ====================="<<std::endl;
        std::cout<<"username: "<<lic_items[0].c_str()<<std::endl;
        std::cout<<"email: "<<lic_items[1].c_str()<<std::endl;
        std::cout<<"serial: "<<lic_items[2].c_str()<<std::endl;
        std::cout<<"======================================================"<<std::endl;
    }

    return lic_items[idx];
}
