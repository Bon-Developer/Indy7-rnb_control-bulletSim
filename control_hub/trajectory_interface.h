//
// Created by panda on 20. 9. 19..
//

#ifndef SRC_ONLINE_INTERPOLATOR_H
#define SRC_ONLINE_INTERPOLATOR_H

#define LOG_QOUT false // generates error
#include <mutex>          // std::mutex

///////////////////////////////////////////////////////////////////////////////
/////////////////////// external communication thread /////////////////////////
///////////////////////////////////////////////////////////////////////////////
#include <memory>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <time.h>
#include <sys/types.h>
#ifdef _WIN32
#include <WinSock2.h>
#include <windows.h>
//#include <WS2tcpip.h>
#include <process.h>
#else
#include <pthread.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif
#include <queue>
#include "gason.h"
#include "utils_rnb.h"
#include "controller_interface.h"

#ifdef _WIN32
#pragma comment(lib,"ws2_32")	//Must Link Winsock Library "ws2_32.dll"
#endif

#include "algebra.h"

#define PORT_TRAJ 9980
#define TRAJ_BUF_LEN 1000000
#define STOP_DURATION 0.1
#define DEFAULT_SAMPLEING_PERIOD 2e-2


namespace RNB {

/**
 * @class TrajectoryInterface
 * @biref Trajectory interface for control hub
 * @remark	This receives trajectory data as json format through TCP/IP socket. \n
 * 			C-spline Interpolation is also applied, if the trajectory is sparse.
 * 			The interpolation rule is X_t = Alpha * (t^3, t^2)' + V0*t + X0, with fixed sampling and control frequency.
 */
class TrajectoryInterface {
public:
	double V_SATURATE = 1.7;		/**< @brief saturation velocity */
	double A_SATURATE = 3.14;		/**< @brief saturation acceleration */

	ControllerInterface::ControlMode mode;
	int DIM; 		        /**< @brief In/out trajectory dimension */
	int ADIM;               /**< @brief algebra dimension, if algebra_p is set */
	int DT, PORT;
	Eigen::VectorXd Xcur;	/**< @brief Current position */
	Eigen::VectorXd X0;		/**< @brief Starting waypoint position of current interpolation */
	Eigen::VectorXd V0;		/**< @brief Starting waypoint velocity of current interpolation (in algebra if algebra_p is set) */
	Eigen::MatrixXd Alpha;		/**< @brief Current interpolation coefficient matrix  (in algebra if algebra_p is set)*/
	double time0;		/**< @brief Starting time of current interpolation */
	double _t;			/**< @brief record of current time */

	Eigen::VectorXd Xd;		/**< @brief Memory holder for next position */
	Eigen::VectorXd Vd;		/**< @brief Memory holder for next velocity */
	Eigen::VectorXd Ad;		/**< @brief Memory holder for next acceleration */

	double period_s;		/**< @brief Waypoint sampling period */
	double period_c;			/**< @brief Control period */

	Eigen::MatrixXd TimeMat;	/**< @brief Sampling time matrix (t^3, t^2 ; 8xt^3, 4xt^2)*/
	Eigen::MatrixXd TimeMatInv;	/**< @brief Pre-calculated inverse of the time matrix */
	std::deque<Eigen::VectorXd> Xqueue;			/**< @brief Queue for waypoint position value */
	std::deque<Eigen::VectorXd> Vqueue;			/**< @brief Queue for waypoint velocity value */
	std::deque<Eigen::MatrixXd> Alphaqueue;		/**< @brief Queue for waypoint acceleration value */

#ifdef _WIN32
	HANDLE hMutex; 			/**< @brief mutex for TCP/IP communication thread sync */
	HANDLE hThread; 		/**< @brief TCP/IP communication thread pointer */
	unsigned dwThreadID; 	/**< @brief TCP/IP communication thread ID */
#else
	pthread_mutex_t mtx;           	/**< @brief mutex for critical section */
	pthread_t p_thread = NULL;		/**< @brief TCP/IP communication thread */
#endif
	int thr_id = NULL;				/**< @brief Thread ID */
	bool stop_thread = false;		/**< @brief flag to request stop thread */
	bool thread_stopped = false;	/**< @brief flag to check thread status */

	bool follow_traj=false;			/**< @brief flag to indicate trajectory following status */
	bool pause_traj = false;		/**< @brief pause following trajectory to load trajectory safely */
	double stop_time0=NULL;			/**< @brief flag to stop sequence starting time */

    AlgebraPtr algebra_p;            /**< @brief definition of algebra for interpolation */

public:
	/**
	 * @param DIM:	dimension of the trajectory
	 * @param DT:	trajectory sampling period
	 * @param PORT: trajectory interface port
	 */
	TrajectoryInterface(ControllerInterface::ControlMode mode, int DIM, double DT, int PORT=NULL);

	~TrajectoryInterface();

	void set_algebra(AlgebraPtr algebra_p){
        if (DIM!=algebra_p->dim){
            throw("Algebra input dimension does not match the trajectory dimension");
        }
	    this->algebra_p = algebra_p;
	    ADIM = algebra_p->alg_dim;
	}

	/**
	 * @brief calculate interpolation matrix by inverting equation (x1, x2) = (a*t^3 + b*t^2 + v0*t + x0, a*(2t)^3 + b*(2t)^2 + v0*(2t) + x0)
	 * @param x0 initial value (DIM, 1)
	 * @param v0 initial velocity (in algebra if algebra_p is set) (ADIM, 1)
	 * @param x1 next value (DIM, 1)
	 * @param x2 next next value (DIM, 1)
	 * @return interpolation matrix (ADIM, 2). X_t = Alpha * (t^3, t^2)' + V0*t + X0
	 */
	Eigen::MatrixXd calc_alpha(Eigen::VectorXd & X0, Eigen::VectorXd & V0, Eigen::VectorXd X1, Eigen::VectorXd X2);

	/**
	 * @brief calculate interpolated value and saturate it by V_SATURATE and A_SATURATE
	 * @param t		time passed from X0
	 * @param X0	initial position
	 * @param V0	initial velocity (in algebra if algebra_p is set)
	 * @param Alpha	interpolation matrix (ADIM, 2). X_t = Alpha * (t^3, t^2)' + V0*t + X0
	 * @param pd	position output
	 * @param vd	velocity output
	 * @param ad	acceleration output
	 */
	void calc_xva(double t, Eigen::VectorXd & X0, Eigen::VectorXd & V0, Eigen::MatrixXd & Alpha,
			Eigen::VectorXd &pd, Eigen::VectorXd &vd, Eigen::VectorXd &ad);

	/**
	 * @brief push next waypoint sample
	 */
	int push_next_qs(double *qs_next);

	/**
	 * @brief Update current position. To give information to the user and set reference position when reset.
	 */
	void update_position(const double *p);

	/**
	 * @brief Get current position to a double array.
	 */
	void get_qcur(double *qcur);

	/**
	 * @brief Get next target position, velocity, and acceleration.
	 */
	void get_next_qc(double time, double * _pd, double * _vd, double * _ad);

	/**
	 * @brief reset trajectory and stop following
	 */
	int reset_traj(double _period_c, double _period_s);

	/**
	 * @brief Set sampling period and calculte inverse of time matrix in advance.
	 */
	int set_sampling_period(double _period_s);

	/**
	* @brief Empty all queue and stop at current position.
	*/
	bool follow();

	/**
	* @brief pause following trajectory - to update trajectory safely without calling stop()
	* @param pause	true: pause, false: resume
	*/
	bool pause(bool pause_traj);

	/**
	* @brief Mark stop time at the end of current queue.
	*/
	bool stop();

	void init_thread(double _period_c, double _period_s);
};

typedef std::shared_ptr<TrajectoryInterface> TrajectoryInterfacePtr;

/**
 * socket thread fuction for trajectory interface
 */
#ifdef _WIN32
unsigned __stdcall traj_socket_thread_fun(void *arg);
#else
void *traj_socket_thread_fun(void *arg);
#endif
}
#endif //SRC_ONLINE_INTERPOLATOR_H
