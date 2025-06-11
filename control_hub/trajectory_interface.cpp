/**
 * trajectory_interface.cpp
 *
 *  Created on: 2021. 3. 8.
 *      Author: RNB_CAD
 */
#include "trajectory_interface.h"

using namespace RNB;

#define _USE_MUTEX


RNB::TrajectoryInterface::TrajectoryInterface(
		ControllerInterface::ControlMode mode, int DIM, double DT, int PORT) :
		mode(mode), DIM(DIM), ADIM(DIM), DT(DT),
		Xcur(DIM), X0(DIM), V0(DIM), Alpha(DIM, 2),
		time0(0), _t(0),
		Xd(DIM), Vd(DIM), Ad(DIM),
		TimeMat(2,2), TimeMatInv(2,2),
        algebra_p(nullptr)
{
	this->PORT = PORT != NULL ? PORT : PORT_TRAJ + mode;
#ifdef _WIN32
	WSADATA wsaData;

	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
		printf("WSAStartup() error!");

#ifdef _USE_MUTEX
	hMutex = CreateMutex(NULL, FALSE, NULL);
	if (hMutex == NULL) {
		std::cout << ANSI_COLOR_RED"======================= ControlHub: CreateMutex() Error ========================\n" ANSI_COLOR_RESET << std::endl;
		return;
#endif
	}
#else
#ifdef _USE_MUTEX
	pthread_mutex_init(&mtx, NULL);
#endif
#endif
	init_thread(DT, DEFAULT_SAMPLEING_PERIOD);
}

RNB::TrajectoryInterface::~TrajectoryInterface(){
	stop_thread = true;
	while(!thread_stopped){
		sleep_microseconds(10000);
	}
#ifdef _WIN32
	CloseHandle(hThread);
	WSACleanup();
#else
#ifdef _USE_MUTEX
	pthread_mutex_destroy(&mtx);
#endif
#endif
	std::cout << ANSI_COLOR_CYAN "================== TrajectoryInterface Destroyed ==================" ANSI_COLOR_RESET << std::endl;
}


Eigen::MatrixXd RNB::TrajectoryInterface::calc_alpha(
        Eigen::VectorXd & X0, Eigen::VectorXd & V0, Eigen::VectorXd X1, Eigen::VectorXd X2) {
	Eigen::MatrixXd Alpha(ADIM, 2);
	Eigen::MatrixXd Pmat(ADIM, 2);

	Eigen::VectorXd dY0(ADIM), dY1(ADIM), dY2(ADIM);
    if(algebra_p != nullptr) {
        Eigen::MatrixXd Xlist(DIM, 3);
        Xlist.block(0, 0, DIM, 1) << X0;
        Xlist.block(0, 1, DIM, 1) << X1;
        Xlist.block(0, 2, DIM, 1) << X2;
        Eigen::MatrixXd dYlist = algebra_p->diff_in_alg(X0, Xlist);
        dY0 << dYlist.block(0, 0, ADIM, 1);
        dY1 << dYlist.block(0, 1, ADIM, 1);
        dY2 << dYlist.block(0, 2, ADIM, 1);
    }
    else{
        dY0 << X0-X0;
        dY1 << X1-X0;
        dY2 << X2-X0;
    }
    Pmat.block(0, 0, ADIM, 1)  << (dY1 - dY0 - V0 * period_s);
    Pmat.block(0, 1, ADIM, 1) << (dY2 - dY0 - 2 * V0 * period_s);
	Alpha << Pmat * TimeMatInv.transpose();
	return Alpha;
}


void RNB::TrajectoryInterface::calc_xva(double t, Eigen::VectorXd & X0, Eigen::VectorXd & V0,
		Eigen::MatrixXd & Alpha, Eigen::VectorXd &pd, Eigen::VectorXd &vd, Eigen::VectorXd &ad) {
	Eigen::Vector2d t_32, t_21, t_10;
	t_32 << pow(t, 3), pow(t, 2);
	t_21 << pow(t, 2) * 3, t * 2;
	t_10 << t * 6, 2;

	if(algebra_p != nullptr){
	    pd << algebra_p->add_from_alg(X0, (Alpha * t_32 + V0 * t));
	}
	else{
        pd << (Alpha * t_32 + V0 * t + X0);
	}
    vd << (Alpha * t_21 + V0);
    ad << (Alpha * t_10);


//	vd = vd.cwiseMax(-V_SATURATE).cwiseMin(V_SATURATE);
//	ad = vd.cwiseMax(-A_SATURATE).cwiseMin(A_SATURATE);
}


int RNB::TrajectoryInterface::push_next_qs(double *qs_next) {
	Eigen::VectorXd Xnew(DIM);
	double_to_vector(qs_next, Xnew);

#ifdef _USE_MUTEX
#ifdef _WIN32
	WaitForSingleObject(
		hMutex,    // handle to mutex
		INFINITE);  // no time-out interval
#else
	pthread_mutex_lock(&mtx);
#endif
#endif
	int qcount = Xqueue.size();
	int alpha_count = Alphaqueue.size();
	if (qcount < 2) {
		Xqueue.push_back(Xnew); // --> Xqueue[2]
#ifdef _USE_MUTEX
#ifdef _WIN32
		ReleaseMutex(hMutex);
#else
		pthread_mutex_unlock(&mtx);
#endif
#endif
		return qcount;
	}

	if (qcount - alpha_count>2) {
		Eigen::VectorXd X0tmp, V0tmp;
		Eigen::MatrixXd Alpha0tmp;
		if (alpha_count==0){
			X0tmp = X0;
			V0tmp = V0;
			Alpha0tmp = Alpha;
		}
		else{
			X0tmp = Xqueue[alpha_count-1];
			V0tmp = Vqueue[alpha_count-1];
			Alpha0tmp = Alphaqueue[alpha_count-1];
		}
		Eigen::VectorXd Xque0 = Xqueue[alpha_count];
		Eigen::VectorXd Xque1 = Xqueue[alpha_count+1];
		Eigen::VectorXd Xque2 = Xqueue[alpha_count+2];

		Eigen::VectorXd Xtmp(DIM);
		Eigen::VectorXd Vtmp(ADIM);
		Eigen::VectorXd Atmp(ADIM);
		calc_xva(period_s, X0tmp, V0tmp, Alpha0tmp, Xtmp, Vtmp, Atmp); // calculate next X, V, A
//		Xqueue[alpha_count] << Xtmp; // theoretically next Xqueue and the calculated X value should be the same

		Eigen::MatrixXd Alphatmp(ADIM,2);
        Alphatmp << calc_alpha(Xque0, Vtmp, Xque1, Xque2); // calculate Alpha for next interpolation

		Vqueue.push_back(Vtmp); // --> Vqueue[0]
		Alphaqueue.push_back(Alphatmp); // --> Alphaqueue[0]
		Xqueue.push_back(Xnew); // --> Xqueue[2]
	}
	else{
		Xqueue.push_back(Xnew); // --> Xqueue[2]
	}
#ifdef _USE_MUTEX
#ifdef _WIN32
	ReleaseMutex(hMutex);
#else
	pthread_mutex_unlock(&mtx);
#endif
#endif
	return qcount;
}


void RNB::TrajectoryInterface::update_position(const double *p) {
	memcpy(Xcur.data(), p, DIM * sizeof(double));
}


void RNB::TrajectoryInterface::get_qcur(double *qcur) {
	memcpy(qcur, Xcur.data(), DIM * sizeof(double));
}


void RNB::TrajectoryInterface::get_next_qc(double time, double * _pd, double * _vd, double * _ad) {
	// pthread_mutex_lock(&mtx);
	_t = time;
	if (follow_traj){
        if (stop_time0 != NULL){
            if (time > stop_time0+STOP_DURATION){
                stop_time0 = NULL;
                follow_traj = false;
				pause_traj = false;
            }
        }
		if (pause_traj) {
			Xd << X0;
			Vd.setZero();
			Ad.setZero();
			vector_to_double(Xd, _pd);
			vector_to_double(Vd, _vd);
			vector_to_double(Ad, _ad);
			return;
		}
		if (time - time0 > period_s) { // time exceeds current trajectory period
			if (!Xqueue.empty()) { // if trajectory is not empty
				Eigen::VectorXd Xtmp = Xqueue.front();
				Xqueue.pop_front();
				if (!(Vqueue.empty() || Alphaqueue.empty())) { // following trajectory is fully prepared
					X0 = Xtmp;
					V0 = Vqueue.front();
					Alpha = Alphaqueue.front();
					Vqueue.pop_front();
					Alphaqueue.pop_front();
				} else { // following trajectory is not full-prepared (required number of following points are not recieved)
					calc_xva(period_s, X0, V0, Alpha, Xd, Vd, Ad);
					X0 = Xtmp;
					for (int i_dim = 0; i_dim < DIM; i_dim++) {
						V0(i_dim, 0) = Vd(i_dim, 0) / 2;
						Alpha(i_dim, 0) = 0;
						Alpha(i_dim, 1) = 0;
					}
				}
			} else { // if trajectory is empty, use current trajectory and set following trajectory constant
				calc_xva(period_s, X0, V0, Alpha, Xd, Vd, Ad);
				for (int i_dim = 0; i_dim < DIM; i_dim++) {
					X0(i_dim, 0) = Xd(i_dim, 0);
					V0(i_dim, 0) = 0;
					Alpha(i_dim, 0) = 0;
					Alpha(i_dim, 1) = 0;
				}
			}
			time0 = period_s * floor(time / period_s); // set time0 for next period
		}
		calc_xva(time - time0, X0, V0, Alpha, Xd, Vd, Ad); // calculate trajectory for current time
	}
	vector_to_double(Xd, _pd);
	vector_to_double(Vd, _vd);
	vector_to_double(Ad, _ad);
	// pthread_mutex_unlock(&mtx);
}


int RNB::TrajectoryInterface::reset_traj(double _period_c, double _period_s) {
#ifdef _USE_MUTEX
#ifdef _WIN32
	WaitForSingleObject(
		hMutex,    // handle to mutex
		INFINITE);  // no time-out interval
#else
	pthread_mutex_lock(&mtx);
#endif
#endif
	for (int i_dim = 0; i_dim < DIM; i_dim++) {
		time0 = 0;
		X0(i_dim, 0) = Xcur[i_dim];
		V0(i_dim, 0) = 0;
		Alpha(i_dim, 0) = 0;
		Alpha(i_dim, 1) = 0;
		Xd(i_dim, 0) = Xcur[i_dim];
		Vd(i_dim, 0) = 0;
		Ad(i_dim, 0) = 0;
	}
	Xqueue.clear();
	Vqueue.clear();
	Alphaqueue.clear();
	period_c = _period_c;
	follow_traj = false;
	pause_traj = false;
#ifdef _USE_MUTEX
#ifdef _WIN32
	ReleaseMutex(hMutex);
#else
	pthread_mutex_unlock(&mtx);
#endif
#endif
	return set_sampling_period(_period_s);
}


int RNB::TrajectoryInterface::set_sampling_period(double _period_s) {
#ifdef _USE_MUTEX
#ifdef _WIN32
	WaitForSingleObject(
		hMutex,    // handle to mutex
		INFINITE);  // no time-out interval
#else
	pthread_mutex_lock(&mtx);
#endif
#endif
	period_s = _period_s;
	TimeMat << period_s * period_s * period_s, period_s * period_s, 8 * period_s
			* period_s * period_s, 4 * period_s * period_s;
	TimeMatInv << TimeMat.inverse();
#ifdef _USE_MUTEX
#ifdef _WIN32
	ReleaseMutex(hMutex);
#else
	pthread_mutex_unlock(&mtx);
#endif
#endif
	return round(period_s / period_c);
}

bool RNB::TrajectoryInterface::follow() {
	stop_time0 = NULL;
	follow_traj = true;
	pause_traj = false;
	return follow_traj;
}

bool RNB::TrajectoryInterface::pause(bool pause_traj) {
	this->pause_traj = pause_traj;
	return this->pause_traj;
}


bool RNB::TrajectoryInterface::stop() {
	if(stop_time0!=NULL){
		return true;
	}
#ifdef _USE_MUTEX
#ifdef _WIN32
	WaitForSingleObject(
		hMutex,    // handle to mutex
		INFINITE);  // no time-out interval
#else
	pthread_mutex_lock(&mtx);
#endif
#endif
	Eigen::VectorXd Vtmp(DIM);
	Eigen::MatrixXd Alphatmp(DIM,2);
	for (int i_dim = 0; i_dim < DIM; i_dim++) {
		Vtmp(i_dim, 0) = 0;
		Alphatmp(i_dim, 0) = 0;
		Alphatmp(i_dim, 1) = 0;
	}
	Vqueue.push_back(Vtmp);
	Alphaqueue.push_back(Alphatmp);
	if (Xqueue.empty()) {
		Xqueue.push_back(X0);
	} else {
		Xqueue.push_back(Xqueue.back());
	}
	Vqueue.push_back(Vtmp);
	Alphaqueue.push_back(Alphatmp);
	stop_time0 = _t + Xqueue.size()*period_s;
#ifdef _USE_MUTEX
#ifdef _WIN32
	ReleaseMutex(hMutex);
#else
	pthread_mutex_unlock(&mtx);
#endif
#endif
	return true;
}


void RNB::TrajectoryInterface::init_thread(double _period_c, double _period_s) {
	std::cout << "=========== [Trajectory Server] Start trajectory interface thread =============" << std::endl;

	reset_traj(_period_c, _period_s);

	if (thr_id == NULL) {
#ifdef _WIN32
		hThread = (HANDLE)_beginthreadex(NULL, 0, traj_socket_thread_fun,
			this, 0, &dwThreadID);
#else
		thr_id = pthread_create(&p_thread, NULL, traj_socket_thread_fun,
				(void *) this);
#endif
		if (thr_id < 0) {
			printf(
					ANSI_COLOR_RED "[Trajectory Server] thread create error" ANSI_COLOR_RESET);
			exit(0);
		}
	}
}

#ifdef _WIN32
unsigned __stdcall RNB::traj_socket_thread_fun(void *arg) {
#else
void *RNB::traj_socket_thread_fun(void *arg) {
#endif
	TrajectoryInterface *jpr;
	jpr = (TrajectoryInterface *) arg;
	int DIM = jpr->DIM;
	jpr->stop_thread = false;
	jpr->thread_stopped = false;

	std::string wbuffer;
	char buffer[TRAJ_BUF_LEN];
#if defined _WIN32
	SOCKADDR_IN server_addr, client_addr;
#else
	struct sockaddr_in server_addr, client_addr;
#endif
	char temp[32];
	int server_fd, client_fd;
	//server_fd, client_fd

	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		printf(
				ANSI_COLOR_RED "[Trajectory Server] Can't open stream socket\n" ANSI_COLOR_RESET);
		exit(0);
	}

	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(jpr->PORT);

#ifdef _WIN32
	ULONG l;
	l = 1;
#else
	struct linger l;
	l.l_onoff = 1;
	l.l_linger = 0;
#endif
    setsockopt(server_fd,                // SOCKET
            SOL_SOCKET,                // level
            SO_REUSEADDR,            // Option
			(char *) & l,    // Option Value
			sizeof (l));             // Option length

	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = SOCK_TIMEOUT_MS * 1000;

#ifdef _WIN32
	DWORD tv_w;
	tv_w = SOCK_TIMEOUT_MS;
	setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv_w, sizeof tv_w);
#else
	setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
#endif

	if (bind(server_fd, (struct sockaddr *) &server_addr, sizeof(server_addr))
			< 0) { //bind()
		printf(
				ANSI_COLOR_RED "[Trajectory Server] Can't bind local address.\n" ANSI_COLOR_RESET);
#ifdef _WIN32
		return 0;
#else
		return nullptr;
#endif
	}

	if (listen(server_fd, 5) < 0) {
		printf(
				ANSI_COLOR_RED "[Trajectory Server] Can't listening connect.\n" ANSI_COLOR_RESET);
		exit(0);
	}
	printf(
			ANSI_COLOR_CYAN "[Trajectory Server] wating connection request.\n" ANSI_COLOR_RESET);


	fd_set read_fds, tmp_fds;
	FD_ZERO(&read_fds);
	FD_SET(server_fd, &read_fds);
	while (!jpr->stop_thread) {
		int client_len, n;
		tmp_fds = read_fds;
		n = select(server_fd + 1, &tmp_fds, NULL, NULL, &tv);
		if (!n) continue;

		JsonValue read_json;
		JsonAllocator allocator;
		char* endptr;
		std::map<std::string, std::string> send_json_map;
		std::string errs;
		client_len = sizeof(client_addr);

		client_fd = accept(server_fd, (struct sockaddr *) &client_addr,
				reinterpret_cast<__socklen_t *>(&client_len));
		//inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, temp, sizeof(temp));

//        printf(ANSI_COLOR_BLUE"[Trajectory Server] %s client connected.\n" ANSI_COLOR_RESET, temp);

		memset(buffer, 0x00, sizeof(buffer));
		if ((n = __read_sock(client_fd, buffer, TRAJ_BUF_LEN)) <= 0) {
			__close_sock(client_fd);
			continue;
		}

//        printf(ANSI_COLOR_BLUE "[Trajectory Server] read %d bytes.\n" ANSI_COLOR_RESET, (int) msg_size);
//        printf(buffer);
//        printf("\n");

		int parsingRet = jsonParse(buffer, &endptr, &read_json, allocator);

		if (parsingRet != JSON_OK) {
			printf(
					ANSI_COLOR_RED "[Trajectory Server]  parse error %s at $zd.\n" ANSI_COLOR_RESET,
					jsonStrError(parsingRet), endptr - buffer);
			continue;
		}

		int step_c;
		Eigen::VectorXd pd(DIM);
		Eigen::VectorXd vd(DIM);
		Eigen::VectorXd ad(DIM);
		int qcount;
		bool ret;

		bool _reset = false;

		Eigen::VectorXd _qval(DIM);
		double _period_s;

		for (auto i : read_json) {
			if (strcmp(i->key, "reset") == 0) {
				_reset = true;
			} else if (strcmp(i->key, "period_s") == 0) {
				_period_s = i->value.toNumber();
			} else if (strcmp(i->key, "qcur") == 0) {
				int i_dim = 0;
				for (auto i_q : i->value) {
					_qval[i_dim] = i_q->value.toNumber();
					i_dim++;
				}
			} else if (strcmp(i->key, "follow") == 0) {
				ret = jpr->follow();
				send_json_map.insert(std::make_pair("follow", ret ? "true" : "false"));
			} else if (strcmp(i->key, "pause") == 0) {
				ret = jpr->pause(true);
				send_json_map.insert(std::make_pair("pause", ret ? "true" : "false"));
			} else if (strcmp(i->key, "resume") == 0) {
				ret = jpr->pause(false);
				send_json_map.insert(std::make_pair("resume", ret ? "true" : "false"));
			}
			else if (strcmp(i->key, "stop") == 0) {
				ret = jpr->stop();
				send_json_map.insert(std::make_pair("stop", "true"));
			} else if (strcmp(i->key, "terminate") == 0) {
				jpr->stop_thread = true;
				send_json_map.insert(std::make_pair("terminate", ret ? "true" : "false"));
			} else if (strcmp(i->key, "getq") == 0) {
				jpr->get_qcur(_qval.data());
				std::string qval_s("[");

				for (int i_dim = 0; i_dim < DIM; i_dim++) {
					qval_s += to_string_with_precision(_qval[i_dim], 4);
					qval_s += (i_dim + 1 < DIM ? ", " : "]");
				}
				send_json_map.insert(std::make_pair("qval", qval_s));
			} else if (strcmp(i->key, "qval") == 0) {
				int i_dim = 0;
				for (auto i_q : i->value) {
					_qval[i_dim] = i_q->value.toNumber();
					i_dim++;
				}
				qcount = jpr->push_next_qs(_qval.data());
				send_json_map.insert(
						std::make_pair("qcount", std::to_string(qcount)));
			} else if (strcmp(i->key, "qcount") == 0) {
				send_json_map.insert(
						std::make_pair("qcount",
								std::to_string(jpr->Xqueue.size())));
			}
		}
		if (_reset) {
			step_c = jpr->reset_traj(jpr->period_c, _period_s);
			send_json_map.insert(
					std::make_pair("step_c", std::to_string(step_c)));
		}

		wbuffer = "{";
		for (auto itor = send_json_map.begin(); itor != send_json_map.end();
				itor++) {
			wbuffer += "\"" + itor->first + "\": ";
			wbuffer += itor->second;
			wbuffer += ", \n";
		}
		int idx_last_comma = wbuffer.rfind(",");
		wbuffer = wbuffer.substr(0, idx_last_comma) + "}";

//        printf("[Trajectory Server] return %d bytes.\n", wbuffer.length());
		__write_sock(client_fd, wbuffer.data(), wbuffer.length());
		__close_sock(client_fd);
//        printf("[Trajectory Server] %s client closed.\n", temp);
	}
	__close_sock(server_fd);
	printf(
			ANSI_COLOR_CYAN "[Trajectory Server] socket connection closed.\n" ANSI_COLOR_RESET);
	printf(ANSI_COLOR_RESET "\n=============================================================================\n");

	jpr->thread_stopped = true;
	return 0;
}
