/**
 * log_utils.h
 *
 *  Created on: 2021. 3. 8.
 *      Author: RNB_CAD
 */

#ifndef CONTROL_HUB_UTILS_H_
#define CONTROL_HUB_UTILS_H_

#include <string>
#include <sstream>
#include "../3rd_party/Eigen/Eigen"

#define ANSI_COLOR_RESET        "\x1b[0m"
#define ANSI_BOLD               "\x1b[1m"
#define ANSI_COLOR_RED          "\x1b[31m"
#define ANSI_COLOR_GREEN        "\x1b[32m"
#define ANSI_COLOR_YELLOW       "\x1b[33m"
#define ANSI_COLOR_BLUE         "\x1b[34m"
#define ANSI_COLOR_MAGENTA      "\x1b[35m"
#define ANSI_COLOR_CYAN         "\x1b[36m"

#define SOCK_TIMEOUT_MS 200

#ifdef _WIN32
#define sleep_microseconds(usecs) std::this_thread::sleep_for(std::chrono::microseconds(usecs))	
#define __socklen_t int
#define __read_sock(sock, buf, count) recv(sock, buf, count, 0)
#define __write_sock(sock, buf, count) send(sock, buf, count, 0)
#define __close_sock closesocket
#else
#define sleep_microseconds(usecs) usleep(usecs)	
#define __socklen_t socklen_t
#define __read_sock(sock, buf, count) read(sock, buf, count)
#define __write_sock(sock, buf, count) write(sock, buf, count)
#define __close_sock close
#endif

namespace RNB {
/**
 * @fn		double_to_vector
 * @brief	fill Eigen::VectorXd with double array
 * @param 	data_p 	pointer to first item of double array (or, double array itself)
 * @param 	vector	vector to which the data will be filled
 */
inline void double_to_vector(const double* data_p, Eigen::VectorXd& vector){
    memcpy(vector.data(), data_p,
           (unsigned long) vector.size() * sizeof(double));
}

/**
 * @fn		vector_to_double
 * @brief	fill double array with Eigen::VectorXd data
 * @param 	vector	vector from which data will be copied
 * @param 	data_p 	pointer to double array which the data will be filled
 */
inline void vector_to_double(Eigen::VectorXd& vector, double* data_p){
    memcpy(data_p, vector.data(),
           (unsigned long) vector.size() * sizeof(double));
}

/**
 * @fn		double_to_matrix
 * @brief	fill Eigen::MatrixXd with double array
 * @param 	data_p 	pointer to first item of double array (or, double array itself)
 * @param 	matrix	matrix to which the data will be filled
 */
inline void double_to_matrix(const double* data_p, Eigen::MatrixXd& matrix) {
    memcpy(matrix.data(), data_p,
           (unsigned long) matrix.size() * sizeof(double));
}

/**
 * @brief rotation matrix to zyx angles - caution: axis order : z, y, x
 */
inline Eigen::Vector3d Rot2zyx(const Eigen::Matrix3d &R){
    Eigen::Vector3d zyx;
    double sy;
    sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    double x, y, z;

    if (sy > 1e-10) {
        x = atan2(R(2, 1), R(2, 2));
        y = atan2(-R(2, 0), sy);
        z = atan2(R(1, 0), R(0, 0));
    }
    else {
        x = atan2(-R(1, 2), R(1, 1));
        y = atan2(-R(2, 0), sy);
        z = 0;
    }
    zyx << z, y, x;
    return zyx;
}

/**
 * @brief rotation matrix to zyx angles - caution: axis order : z, y, x
 */
inline Eigen::Matrix3d Rot_axis( int axis, double q ){
    Eigen::Matrix3d R;
    if(axis==1){
        R <<
          1,0,0,
                0,cos(q),-sin(q),
                0,sin(q),cos(q);
    }
    if(axis==2){
        R <<
          cos(q),0,sin(q),
                0,1,0,
                -sin(q),0,cos(q);
    }
    if(axis==3){
        R <<
          cos(q),-sin(q),0,
                sin(q),cos(q),0,
                0,0,1;
    }
    return R;
}

inline Eigen::Matrix3d Rot_zyx(double w, double v, double u){
    return Rot_axis(3,w)*Rot_axis(2,v)*Rot_axis(1,u);
}

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 2)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}
}

#endif /* CONTROL_HUB_UTILS_H_ */
