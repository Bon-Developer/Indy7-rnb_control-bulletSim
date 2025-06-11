//
// Created by rnb on 21. 11. 2..
//

#ifndef TEST_CONTROLLER_TRAJ_GEN_H
#define TEST_CONTROLLER_TRAJ_GEN_H

#include <map>
#include <string>
#include <memory>
#include <iostream>
#include <fstream>

#include <Eigen/Eigen>

#define PI 3.1415926

Eigen::MatrixXd get_step_traj(int dims, int length,  int step_start, Eigen::VectorXd q_0, Eigen::VectorXd q_1){
    Eigen::MatrixXd traj(length, dims);
    for(int i=0; i<length; i++) {
        if(i < step_start){
            traj.block(i, 0, 1, dims) << q_0.transpose();
        }
        else{
            traj.block(i,0,1,dims) << q_0.transpose();
        }
    }
    return traj;
}

Eigen::MatrixXd get_step_traj(int dims, int length,  int step_start, Eigen::VectorXd q_0, double value_1){
    Eigen::VectorXd q_1(dims);
    q_1.fill(value_1);
    return get_step_traj(dims, length, step_start, q_0, q_1);
}

Eigen::MatrixXd get_sin_traj(int dims, int length,  int period_steps, Eigen::VectorXd q_0, Eigen::VectorXd mag){
    Eigen::MatrixXd traj(length, dims);
    for(int i=0; i<length; i++) {
        double sin_val = sin(PI * 2 / period_steps * i);
        Eigen::VectorXd q = mag * sin_val;
        traj.block(i,0,1,dims) << q.transpose();
    }
    return traj;
}

Eigen::MatrixXd get_sin_traj(int dims, int length,  int period_steps, Eigen::VectorXd q_0, double mag) {
    Eigen::VectorXd q_mag(dims);
    q_mag.fill(mag);
    return get_sin_traj(dims, length, period_steps, q_0, q_mag);
}

Eigen::VectorXd differentitate(Eigen::VectorXd q0, Eigen::VectorXd q1, double dt) {
    Eigen::VectorXd qdiff(q0.size());
    qdiff = q1 -  q0;
    qdiff /= dt;
    return qdiff;
}

Eigen::MatrixXd differentitate(Eigen::MatrixXd traj, double dt){
    int length = traj.rows();
    int dims = traj.cols();
    Eigen::MatrixXd traj_vel(length, dims);
    traj_vel.block(0, 0, 1, dims).fill(0);
    for(int i=1; i<length; i++) {
        traj_vel.block(i, 0, 1, dims) << differentitate(
                traj.block(i-1, 0, 1, dims).transpose(),
                traj.block(i, 0, 1, dims).transpose(),
                dt).transpose() ;
    }
    return traj_vel;
}

void save_mat(std::string filepath, Eigen::MatrixXd matrix){
    std::ofstream myfile;
    myfile.open (filepath);
    int dims = matrix.cols();
    for(int i=0; i< matrix.rows(); i++){
        for(int j=0;j<matrix.cols();j++){
            myfile << matrix(i, j);
            if(j<matrix.cols()-1) myfile << ",";
        }
        myfile << "\n";
    }
    myfile.close();
}

#endif //TEST_CONTROLLER_TRAJ_GEN_H
