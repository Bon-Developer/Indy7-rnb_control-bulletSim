/*
 * indyutils.h
 *
 *  Created on: 2021. 3. 6.
 *      Author: RNB_CAD
 */

#ifndef INDYUTILS_H_
#define INDYUTILS_H_

#define LICENSE_FILE "./Components/sdk_license.lic"

#include <Eigen/Eigen>

/**
 * @brief read license from "./Components/sdk_license.lic". idx=0: username, idx=1: email, idx2: serial
 */
std::string read_license(int idx);


/**
 * @brief change P and R components of vector or matrix
 */
template<typename T>
T flip_pr(const T x){
	T y;
	int row_dim = x.cols();
	y.block(0,0,3,row_dim) << x.block(3,0,3,row_dim);
	y.block(3,0,3,row_dim) << x.block(0,0,3,row_dim);
	return y;
}

#endif /* INDYUTILS_H_ */
