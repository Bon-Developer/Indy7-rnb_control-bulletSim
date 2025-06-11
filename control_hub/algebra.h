//
// Created by rnb on 21. 12. 29..
//

#ifndef BULLETSIMCONTROL_ALGEBRA_H
#define BULLETSIMCONTROL_ALGEBRA_H

#include "../3rd_party/Eigen/Eigen"
#include "../3rd_party/unsupported/Eigen/MatrixFunctions"
#include "utils_rnb.h"

namespace RNB{
    class Algebra{
    public:
        int dim, alg_dim;
        Algebra(){}
        Algebra(int dim):dim(dim), alg_dim(dim){}

        /**
         * @brief calculate difference in algebra
         * @param X0    reference vector (dim, 1)
         * @param Xlist list of vectors to calculate difference (Xlist - X0)
         * @return difference in algebra (alg_dim, n)
         */
        virtual Eigen::MatrixXd diff_in_alg(Eigen::VectorXd X0, Eigen::MatrixXd Xlist) = 0;

        /**
         * @brief add difference in algebra
         * @param X0    reference vector (dim, 1)
         * @param Ylist list of vectors to add in algebra (alg_dim, n)
         * @return added vectors (dim, n)
         */
        virtual Eigen::MatrixXd add_from_alg(Eigen::VectorXd X0, Eigen::MatrixXd dYlist) = 0;
    };

    typedef std::shared_ptr<Algebra> AlgebraPtr;

    class Euclidean: public Algebra{
    public:
        Euclidean(int dim): Algebra(3){}

        Eigen::MatrixXd diff_in_alg(Eigen::VectorXd X0, Eigen::MatrixXd Xlist) override {
            return Xlist.colwise() - X0;
        }

        Eigen::MatrixXd add_from_alg(Eigen::VectorXd X0, Eigen::MatrixXd dYlist) override {
            return dYlist.colwise() + X0;
        }
    };

    class SO3group: public Algebra{
    public:
        SO3group(): Algebra(3){}

        /**
         * @brief mapping function from vector to orientation matrix
         */
        virtual Eigen::Matrix3d to_rmat(const Eigen::Vector3d& X) = 0;

        /**
         * @brief mapping function from orientation matrix to vector
         */
        virtual Eigen::Vector3d from_rmat(const Eigen::Matrix3d& R) = 0;

        /**
         * @brief calculate difference in so3 and rotate it to represent it in spatial frame
         */
        Eigen::MatrixXd diff_in_alg(Eigen::VectorXd X0, Eigen::MatrixXd Xlist) override {
            Eigen::Matrix3d R0 = to_rmat(X0);
            Eigen::Matrix3d R0T = R0.transpose();
            Eigen::MatrixXd Ylist(alg_dim, Xlist.cols());
            Eigen::Matrix3d R1;
            Eigen::Matrix3d R01;
            Eigen::Matrix3d w_hat;
            Eigen::Vector3d w;
            for(int i=0; i< Xlist.cols(); i++){
                R1 = to_rmat(Xlist.block(0,i,dim, 1));
                R01 <<  R0T*R1;
                w_hat = R01.log();
                w << w_hat(2,1), w_hat(0,2),w_hat(1,0);
                w = R0*w;
                Ylist.block(0,i,alg_dim, 1) << w;
            }
            return Ylist;
        }

        /**
         * @brief add vector in so3 return in X0 space.
         * @param dYlist    vectors to add, represented in spatial frame.
         */
        Eigen::MatrixXd add_from_alg(Eigen::VectorXd X0, Eigen::MatrixXd dYlist) override {
            Eigen::MatrixXd R0 = to_rmat(X0);
            Eigen::Matrix3d R0T = R0.transpose();
            Eigen::MatrixXd Xlist = Eigen::MatrixXd(dim, dYlist.cols());
            for(int i=0; i< dYlist.cols(); i++) {
                Eigen::Vector3d w = R0T*dYlist.block(0, i, alg_dim, 1);
                Eigen::Matrix3d w_hat;
                w_hat << 0, -w[2], w[1], w[2], 0, -w[0], -w[1], w[0], 0;
                Eigen::Matrix3d dR = w_hat.exp();
                Eigen::Vector3d rvec = from_rmat(R0 * dR);
                Xlist.block(0, i, dim, 1) << rvec[0], rvec[1], rvec[2];
            }
            return Xlist;
        }
    };

    class RotationUVW: public SO3group{
    public:
        Eigen::Matrix3d to_rmat(const Eigen::Vector3d& X) override {
            return Rot_axis(3, X[2])*Rot_axis(2, X[1])*Rot_axis(1, X[0]);
        }

        Eigen::Vector3d from_rmat(const Eigen::Matrix3d& R)  override {
            return Rot2zyx(R).reverse();
        }
    };

    class Combined: public Algebra{
    public:
        std::list<AlgebraPtr> alg_list;
        /**
         * @param alg_list list of Algebra instance
         */
        Combined(std::list<AlgebraPtr> &alg_list): Algebra(), alg_list(alg_list) {
            std::list<int> dim_list;
            std::list<int> out_dim_list;
            dim = 0;
            alg_dim = 0;
            for (auto it = alg_list.begin(); it != alg_list.end(); it++) {
                dim_list.push_back((*it)->dim);
                out_dim_list.push_back((*it)->alg_dim);
                dim += (*it)->dim;
                alg_dim += (*it)->alg_dim;
            }
        }

        Eigen::MatrixXd diff_in_alg(Eigen::VectorXd X0, Eigen::MatrixXd Xlist) override {
            int N = Xlist.cols();
            Eigen::MatrixXd Ylist(alg_dim, N);
            int dim_accum = 0;
            int out_dim_accum =0;
            int dim_now = 0;
            int out_dim_now = 0;
            for(auto it=alg_list.begin(); it!=alg_list.end();it++){
                dim_now = (*it)->dim;
                out_dim_now= (*it)->alg_dim;
                Ylist.block(out_dim_accum, 0, out_dim_now, N) =
                        (*it)->diff_in_alg(X0.block(dim_accum, 0, dim_now, 1),
                                         Xlist.block(dim_accum, 0,dim_now, N));
                dim_accum += dim_now;
                out_dim_accum += out_dim_now;
            }
            return Ylist;
        }

        Eigen::MatrixXd add_from_alg(Eigen::VectorXd X0, Eigen::MatrixXd dYlist) override {
            int N = dYlist.cols();
            Eigen::MatrixXd Xlist(dim, N);
            int dim_accum = 0;
            int out_dim_accum = 0;
            int dim_now = 0;
            int out_dim_now = 0;
            for(auto it=alg_list.begin(); it!=alg_list.end();it++){
                dim_now = (*it)->dim;
                out_dim_now= (*it)->alg_dim;
                Xlist.block(dim_accum, 0, dim_now, N) =
					(*it)->add_from_alg(X0.block(dim_accum,0,dim_now, 1),
                                     dYlist.block(out_dim_accum, 0, out_dim_now, N)
                        );
                dim_accum += dim_now;
                out_dim_accum += out_dim_now;
            }
            return Xlist;
        }
    };
}
#endif //BULLETSIMCONTROL_ALGEBRA_H
