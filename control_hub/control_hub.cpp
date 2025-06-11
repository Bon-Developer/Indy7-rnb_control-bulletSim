/**
 * control_hub.cpp
 *
 *  Created on: 2021. 3. 3.
 *      Author: RNB_CAD
 */

#include "control_hub.h"
#include <string.h>
#include <iostream>
#include <algorithm>

#include <chrono>

using namespace RNB;

RNB::ControlHub::ControlHub(RNB::ControllerInterface::ControlMode mode, int JOINT_DOF, int TASK_DOF, double DT, 
	int ui_port, int traj_port) :
		mode(mode), JOINT_DOF(JOINT_DOF), TASK_DOF(TASK_DOF), DT(DT), _t(0),
		p(TASK_DOF), p_n(TASK_DOF),
		p_d(TASK_DOF), pdot_d(TASK_DOF), pddot_d(TASK_DOF),
		q(JOINT_DOF), qdot(JOINT_DOF), qddot(JOINT_DOF),
		q_d(JOINT_DOF), qdot_d(JOINT_DOF), qddot_d(JOINT_DOF),
		q_n(JOINT_DOF), qdot_n(JOINT_DOF), qddot_n(JOINT_DOF),
		M(JOINT_DOF, JOINT_DOF), C(JOINT_DOF, JOINT_DOF),
		M_n(JOINT_DOF, JOINT_DOF), C_n(JOINT_DOF, JOINT_DOF),
		tauGrav(JOINT_DOF), tauExt(JOINT_DOF),
		J(TASK_DOF, JOINT_DOF), Jdot(TASK_DOF, JOINT_DOF),
		J_n(TASK_DOF, JOINT_DOF), Jdot_n(TASK_DOF, JOINT_DOF),
		FText(TASK_DOF), torque(JOINT_DOF),
		data_logger(round(LOG_BUFF_LEN_SEC*round(1.0/DT))),
		trajectory_interface(mode, mode==ControllerInterface::JOINT_CONTROL ? JOINT_DOF : TASK_DOF, DT, traj_port)
{
	this->ui_port = ui_port != NULL ? ui_port : PORT_BASE + mode;
	data_logger.add_title("q");
	data_logger.add_title("qdot");
    data_logger.add_title("error"); // qd - q
	data_logger.add_title("e_nr"); // qn - q
	data_logger.add_title("torque");
    data_logger.add_title("Fext");
    data_logger.add_title("tauExt");
    data_logger.add_title("custom_value");
	data_logger.add_title("compute_time_us");
	std::cout << ANSI_COLOR_CYAN "======================== Control Hub initialized =========================\n" ANSI_COLOR_RESET << std::endl;
	switch(mode){
	case ControllerInterface::JOINT_CONTROL:
		std::cout << ANSI_COLOR_CYAN "====================== ControlMode: JOINT_CONTROL ========================\n" ANSI_COLOR_RESET << std::endl;
		break;
	case ControllerInterface::TASK_CONTROL:
		std::cout << ANSI_COLOR_CYAN "======================= ControlMode: TASK_CONTROL ========================\n" ANSI_COLOR_RESET << std::endl;
		break;
	}
	printf(ANSI_COLOR_CYAN "================= %dD Joints / %dD Task space / DT = %f ==============\n" ANSI_COLOR_RESET, JOINT_DOF, TASK_DOF, DT);
	printf(ANSI_COLOR_CYAN "==========================================================================\n" ANSI_COLOR_RESET);


	qddot_n.setZero();
	torque.setZero();

#ifdef _WIN32
	hThread = (HANDLE)_beginthreadex(NULL, 0, RNB::serve_controller_server, 
		this, 0, &dwThreadID);
#else
	thr_id__ui = pthread_create(&p_thread_ui, NULL,
		RNB::serve_controller_server, (void *) this);
#endif
}

RNB::ControlHub::~ControlHub() {
	stop_thread = true;
	while(!thread_stopped){
		sleep_microseconds((long long)(DT*1000000*10));
	}
#ifdef _WIN32
	CloseHandle(hThread);
#endif
	std::cout << ANSI_COLOR_CYAN "================== ControlHub Destroyed ==================" ANSI_COLOR_RESET << std::endl;
}

void RNB::ControlHub::reset_controller(bool reset_traj, bool reset_nom) {

	if(before_first_reset){ // it will reset later anyway
		return;
	}
	updating_controller += 1;
	reset_controller_now = false;

	for(int i=0; i<10; i++){
		if(reset_controller_now){
			break;
		}
		sleep_microseconds((long long)(DT*1000000));
	}

	reset_controller(mode==ControllerInterface::JOINT_CONTROL? q.data() : p.data(), reset_traj, reset_nom);

	updating_controller -= 1;
}

void RNB::ControlHub::reset_controller(double * x0, bool reset_traj, bool reset_nom) {
	updating_controller += 1;
	Eigen::VectorXd x0vec(mode==ControllerInterface::JOINT_CONTROL? JOINT_DOF : TASK_DOF);
    x0vec.setZero();
    double_to_vector(x0, x0vec);

	if (controller_p) {
		use_real_kinematics = controller_p->use_real_kinematics;
		use_nominal_kinematics = controller_p->use_nominal_kinematics;
		use_real_dynamics = controller_p->use_real_dynamics;
		use_nominal_dynamics = controller_p->use_nominal_dynamics;
        if(mode==ControllerInterface::TASK_CONTROL){
            controller_p->set_task_space(alg_p);
			trajectory_interface.set_algebra(alg_p);
        }
		controller_p->reset_control(x0vec);
	}

	if(mode==ControllerInterface::JOINT_CONTROL){
        q_d = x0vec;
        p_d.setZero();
	}
	else{
        q_d.setZero();
        p_d = x0vec;
	}
    qdot_d.setZero();
    qddot_d.setZero();

    pdot_d.setZero();
    pddot_d.setZero();

    if(reset_traj){
        trajectory_interface.update_position(x0vec.data());
        trajectory_interface.reset_traj(DT, DEFAULT_SAMPLEING_PERIOD);
    }
    need_nominal_reset = reset_nom;
	updating_controller -= 1;
	before_first_reset = false;
}

int RNB::ControlHub::calculate_joint_control_torque(
		const double *_q, const double *_qdot, const double *_qddot,
		const double *_q_d, const double *_qdot_d, const double *_qddot_d,
		const double *_q_n, const double *_qdot_n,
		const double *_p, const double *_p_n,
		const double *_M, const double *_C,
		const double *_M_n, const double *_C_n,
		const double *_tauGrav, const double *_tauExt,
		const double *_J, const double *_Jdot,
		const double *_J_n, const double *_Jdot_n,
		const double *_FText,
		double *_qddot_n_out, double *torque_out)
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	double_to_vector(_q, q);
	double_to_vector(_qdot, qdot);
	double_to_vector(_qddot, qddot);
	double_to_vector(_q_d, q_d);
	double_to_vector(_qdot_d, qdot_d);
	double_to_vector(_qddot_d, qddot_d);

	double_to_vector(_q_n, q_n);
	double_to_vector(_qdot_n, qdot_n);
    double_to_vector(_qddot, qddot_n);

	double_to_vector(_p, p);
	double_to_vector(_p_n, p_n);

	double_to_matrix(_M, M);
	double_to_matrix(_C, C);

	double_to_matrix(_M_n, M_n);
	double_to_matrix(_C_n, C_n);

	double_to_matrix(_J, J);
	double_to_matrix(_Jdot, Jdot);
	double_to_matrix(_J_n, J_n);
	double_to_matrix(_Jdot_n, Jdot_n);

	double_to_vector(_tauGrav, tauGrav);
	double_to_vector(_tauExt, tauExt);
	double_to_vector(_FText, FText);
    double_to_vector(_tauGrav, torque);

	Eigen::VectorXd custom_dat(JOINT_DOF);
    custom_dat.setZero();

	if(mode!=ControllerInterface::JOINT_CONTROL){
		_t += DT;
		printf(ANSI_COLOR_RED "Wrong controller call - Selcted mode: JOINT_CONTROL" ANSI_COLOR_RESET);
		throw "Wrong controller call - Selcted mode: JOINT_CONTROL";
	}

	if (updating_controller!=0){
		reset_controller_now = true;
	}
	else { // If controller is updating, do not update output -> use previous input value
	    if(trajectory_interface.follow_traj) {
	        trajectory_interface.get_next_qc(_t, q_d.data(), qdot_d.data(), qddot_d.data());
	    }
		if (controller_p == nullptr) {
			vector_to_double(qddot_n, _qddot_n_out);
			vector_to_double(torque, torque_out);
			printf(ANSI_COLOR_YELLOW "Try accessing controller_p while controller not ready \n" ANSI_COLOR_RESET);
			return 0;
		}

		try{
			controller_p->calculate_joint_control_torque(
					q, qdot, qddot,
					q_d, qdot_d, qddot_d,
					q_n, qdot_n,
					p, p_n,
					M, C,
					M_n, C_n,
					tauGrav, tauExt,
					J, Jdot, J_n, Jdot_n, FText,
					qddot_n, torque, custom_dat);
			std::chrono::steady_clock::time_point computation_end = std::chrono::steady_clock::now();
			Eigen::VectorXd e = q_d-q;
			Eigen::VectorXd t(1);
			t[0] = std::chrono::duration_cast<std::chrono::microseconds>(computation_end - begin).count();
			std::vector<Eigen::VectorXd> log_data;
			log_data.push_back(q);
			log_data.push_back(qdot);
			log_data.push_back(e);
			log_data.push_back(q_n-q);
			log_data.push_back(torque);
            log_data.push_back(FText);
            log_data.push_back(tauExt);
            log_data.push_back(custom_dat);
            log_data.push_back(t);
			data_logger.push_log(_t, log_data);
		}
		catch (const std::exception &exc)
		{
			printf(ANSI_COLOR_RED "============== control algorithm calculation error ==============" ANSI_COLOR_RESET);
			std::cerr << ANSI_COLOR_RED << exc.what() << ANSI_COLOR_RESET << std::endl;
			printf(ANSI_COLOR_RED "=================================================================" ANSI_COLOR_RESET);
		}
		catch (...) {
			printf(ANSI_COLOR_RED "============== control algorithm calculation error ==============" ANSI_COLOR_RESET);
		}
	}
    trajectory_interface.update_position(q.data());

	vector_to_double(qddot_n, _qddot_n_out);
	vector_to_double(torque, torque_out);
	_t += DT;

	// Don't erase below line! This seems stablize time sequence of control loop - without this, position error due to timeout frequently occurs
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//	std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[us]" << std::endl;

	return 0;
}

int RNB::ControlHub::calculate_task_control_torque(
		const double *_p, const double *_p_n,
		const double *_p_d, const double *_pdot_d, const double *_pddot_d,
		const double *_q, const double *_qdot, const double *_qddot,
		const double *_q_n, const double *_qdot_n,
		const double *_M, const double *_C,
		const double *_M_n, const double *_C_n,
		const double *_tauGrav, const double *_tauExt,
		const double *_J, const double *_Jdot,
		const double *_J_n, const double *_Jdot_n,
		const double *_FText,
		double *_qddot_n_out, double *torque_out)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	double_to_vector(_p, p);
	double_to_vector(_p_n, p_n);
	double_to_vector(_p_d, p_d);
	double_to_vector(_pdot_d, pdot_d);
	double_to_vector(_pddot_d, pddot_d);

	double_to_vector(_q, q);
	double_to_vector(_qdot, qdot);
	double_to_vector(_qddot, qddot);
	double_to_vector(_q_n, q_n);
	double_to_vector(_qdot_n, qdot_n);
    double_to_vector(_qddot, qddot_n);

	double_to_matrix(_M, M);
	double_to_matrix(_C, C);

	double_to_matrix(_M_n, M_n);
	double_to_matrix(_C_n, C_n);

	double_to_matrix(_J, J);
	double_to_matrix(_Jdot, Jdot);
	double_to_matrix(_J_n, J_n);
	double_to_matrix(_Jdot_n, Jdot_n);

	double_to_vector(_tauGrav, tauGrav);
	double_to_vector(_tauExt, tauExt);
	double_to_vector(_FText, FText);
    double_to_vector(_tauGrav, torque);

    Eigen::VectorXd custom_dat(JOINT_DOF);
    custom_dat.setZero();

	if(mode!=ControllerInterface::TASK_CONTROL){
		_t += DT;
		printf(ANSI_COLOR_RED "Wrong controller call - Selcted mode: TASK_CONTROL\n" ANSI_COLOR_RESET);
		throw "Wrong controller call - Selcted mode: TASK_CONTROL\n";
	}

	if(trajectory_interface.follow_traj){
		if (trajectory_interface.algebra_p == nullptr) {
			printf(ANSI_COLOR_YELLOW "Try accessing trajectory interface before algebra set\n" ANSI_COLOR_RESET);
			vector_to_double(qddot_n, _qddot_n_out);
			vector_to_double(torque, torque_out);
			return 0;
		}
        trajectory_interface.get_next_qc(_t, p_d.data(), pdot_d.data(), pddot_d.data());
	}

	if (updating_controller!=0){
		reset_controller_now = true;
	}
	else { // If controller is updating, do not update output -> use previous input value
		if (controller_p == nullptr) {
			printf(ANSI_COLOR_YELLOW "Try accessing controller_p while controller not ready \n" ANSI_COLOR_RESET);
			vector_to_double(qddot_n, _qddot_n_out);
			vector_to_double(torque, torque_out);
			return 0;
		}
		if (controller_p->alg_p == nullptr) {
			printf(ANSI_COLOR_YELLOW "Try accessing controller_p interface before algebra set \n" ANSI_COLOR_RESET);
			vector_to_double(qddot_n, _qddot_n_out);
			vector_to_double(torque, torque_out);
			return 0;
		}
		try{
			controller_p->calculate_task_control_torque(
					p, p_n,
					p_d, pdot_d, pddot_d,
					q, qdot, qddot,
					q_n, qdot_n,
					M, C,
					M_n, C_n,
					tauGrav, tauExt,
					J, Jdot, J_n, Jdot_n, FText,
					qddot_n, torque, custom_dat);
            std::chrono::steady_clock::time_point computation_end = std::chrono::steady_clock::now();
            Eigen::VectorXd e;
            if(alg_p!=nullptr){
                e = alg_p->diff_in_alg(p, p_d);
            }
            else{
                e = p_d - p;
            }
            Eigen::VectorXd t(1);
            t[0] = std::chrono::duration_cast<std::chrono::microseconds>(computation_end - begin).count();
			std::vector<Eigen::VectorXd> log_data;
			log_data.push_back(q);
			log_data.push_back(qdot);
			log_data.push_back(e);
			log_data.push_back(p_n-p);
			log_data.push_back(torque);
			log_data.push_back(FText);
            log_data.push_back(tauExt);
            log_data.push_back(custom_dat);
            log_data.push_back(t);
			data_logger.push_log(_t, log_data);
		}
		catch (const std::exception &exc)
		{
			printf(ANSI_COLOR_RED "============== control algorithm calculation error ==============" ANSI_COLOR_RESET);
			std::cerr << ANSI_COLOR_RED << exc.what() << ANSI_COLOR_RESET << std::endl;
			printf(ANSI_COLOR_RED "=================================================================" ANSI_COLOR_RESET);
		}
		catch (...) {
			printf(ANSI_COLOR_RED "============== control algorithm calculation error ==============" ANSI_COLOR_RESET);
		}
	}
    trajectory_interface.update_position(p.data());

	vector_to_double(qddot_n, _qddot_n_out);
	vector_to_double(torque, torque_out);
	_t += DT;
	return 0;
}

void * RNB::change_controller(ControlHub* control_hup_p,
		Dropdown &dropdown_controller, MultipleInputForm &param_setting, void *controller_lib){
	RNB::ControlChangeFun controller_generator;
	std::string control_name_camel;

	control_hup_p->updating_controller += 1;
	bool controller_change_success = true;

	if (controller_lib != NULL) {
		std::cout << ANSI_COLOR_RED "========= unload controller library =========" ANSI_COLOR_RESET
				<< std::endl;
		control_hup_p->controller_p.reset(); // resetting shared pointer to NULL. NOT reset_controller!!
	    try{
#ifdef _WIN32
			FreeLibrary((HINSTANCE)controller_lib);
#else
	    	dlclose(controller_lib);
#endif
	    }
		catch (const std::exception &exc)
		{
			controller_change_success = false;
	        std::cout << ANSI_COLOR_RED "========= ERROR in deleting controller =========" ANSI_COLOR_RESET << std::endl;
		    std::cerr << ANSI_COLOR_RED << exc.what() << ANSI_COLOR_RESET << std::endl;
	        std::cout << ANSI_COLOR_RED "================================================" ANSI_COLOR_RESET << std::endl;
		}
		catch (...) {
			controller_change_success = false;
	        std::cout << ANSI_COLOR_RED "========= ERROR in deleting controller =========" ANSI_COLOR_RESET << std::endl;
		}
	}
	if (dropdown_controller.selected == DEFAULT_CONTROLLER_NAME){
	    try{
			control_hup_p->controller_p = std::make_shared<ControlAlgorithmDefault>(
					control_hup_p->mode, control_hup_p->JOINT_DOF, control_hup_p->TASK_DOF,
					control_hup_p->DT);
			controller_change_success = true;
	    }
		catch (const std::exception &exc)
		{
			controller_change_success = false;
	        std::cout << ANSI_COLOR_RED "========= ERROR in generating controller =========" ANSI_COLOR_RESET << std::endl;
		    std::cerr << ANSI_COLOR_RED << exc.what() << ANSI_COLOR_RESET << std::endl;
	        std::cout << ANSI_COLOR_RED "==================================================" ANSI_COLOR_RESET << std::endl;
		}
		catch (...) {
			controller_change_success = false;
	        std::cout << ANSI_COLOR_RED "========= ERROR in generating controller =========" ANSI_COLOR_RESET << std::endl;
		}
	}
	else {
#ifdef _WIN32
		controller_lib = (void*)LoadLibrary((CONTROLLER_PATH + dropdown_controller.selected + ".dll").c_str());
#else
		controller_lib =
				dlopen(
						(CONTROLLER_PATH + dropdown_controller.selected
								+ ".so").c_str(), RTLD_LAZY);
#endif
		if (controller_lib == NULL) {
			controller_change_success = false;
			std::cout << ANSI_COLOR_RED "========= controller library not loaded =========" ANSI_COLOR_RESET
					<< std::endl;
#ifndef _WIN32
			std::cout << dlerror() << std::endl;
#endif
		} else {
			std::cout << ANSI_COLOR_CYAN "========= controller library loaded =========" ANSI_COLOR_RESET << std::endl;
		}

		if (controller_change_success){
#ifdef _WIN32
			controller_generator =
				(ControlChangeFun) GetProcAddress((HINSTANCE)controller_lib, "create_controller");
#else
			controller_generator =
				(ControlChangeFun)dlsym(controller_lib, "create_controller");
#endif
		}
		else{
			controller_generator = NULL;
		}

		if ( controller_generator == NULL) {
			controller_change_success = false;
			std::cout << ANSI_COLOR_RED <<"========= controller_generator not loaded =========" ANSI_COLOR_RESET <<std::endl;
#ifndef _WIN32
			std::cout << ANSI_COLOR_RED <<dlerror()<< ANSI_COLOR_RESET <<std::endl;
#endif
		} else {
			std::cout << ANSI_COLOR_CYAN "========= controller_generator loaded =========" ANSI_COLOR_RESET << std::endl;
		}


		if (controller_change_success){
		    try{
				controller_generator(
						control_hup_p->mode, control_hup_p->JOINT_DOF, control_hup_p->TASK_DOF,
						control_hup_p->DT, control_hup_p->controller_p);
		    }
			catch (const std::exception &exc)
			{
		        std::cout << ANSI_COLOR_RED "========= ERROR in generating controller =========" ANSI_COLOR_RESET << std::endl;
			    std::cerr << ANSI_COLOR_RED << exc.what() << ANSI_COLOR_RESET << std::endl;
		        std::cout << ANSI_COLOR_RED "==================================================" ANSI_COLOR_RESET << std::endl;
			}
			catch (...) {
		        std::cout << ANSI_COLOR_RED "========= ERROR in generating controller =========" ANSI_COLOR_RESET << std::endl;
				controller_change_success=false;
		    }
		}
	}

	if(controller_change_success){
		std::cout << ANSI_COLOR_CYAN "========= controller initialized =========" ANSI_COLOR_RESET << std::endl;

		switch(control_hup_p->mode){
		case ControllerInterface::JOINT_CONTROL:
			control_name_camel = "JointControl";
					break;
		case ControllerInterface::TASK_CONTROL:
			control_name_camel = "TaskControl";
					break;
		default:
			std::cout << ANSI_COLOR_RED "============== NONE-DEFINED CONTROl MODE ================" ANSI_COLOR_RESET << std::endl;
			throw "NONE-DEFINED CONTROl MODE";
		}

		param_setting.clear();
		param_setting.set_title((dropdown_controller.selected + "-" + control_name_camel).c_str());
		std::cout << control_hup_p->controller_p << std::endl;
		for (auto itor = control_hup_p->controller_p->gain_map.begin();
				itor != control_hup_p->controller_p->gain_map.end();
				itor++) {
			param_setting.add_input(itor->first.c_str(), itor->second);
		}
		std::cout << ANSI_COLOR_CYAN "========= controller generated =========" ANSI_COLOR_RESET << std::endl;

		param_setting.load_params();
		apply_gains(control_hup_p, param_setting, true);
	}
	control_hup_p->updating_controller -= 1;
	return controller_lib;
}

void RNB::apply_gains(ControlHub* control_hup_p, MultipleInputForm &param_setting, bool reset_nom){
	bool value_changed = false;
	control_hup_p->updating_controller += 1;

	std::cout << ANSI_COLOR_CYAN "========== update gains ===========" ANSI_COLOR_RESET << std::endl;
	for (auto itor = control_hup_p->controller_p->gain_map.begin();
			itor != control_hup_p->controller_p->gain_map.end(); itor++) {
		std::string key = itor->first;
		std::vector<double> value_vec = param_setting.value_map[itor->first];
		for(int i_val=0; i_val<value_vec.size(); i_val++){
			std::string value_name = key+std::to_string(i_val);
			double val = value_vec[i_val];
			bool changed_tmp = ((control_hup_p->controller_p->gain_map)[key][i_val]!= val);;
			if(changed_tmp) {
                std::cout << ANSI_COLOR_CYAN << value_name << " : " << val << ANSI_COLOR_RESET << std::endl;
                (control_hup_p->controller_p->gain_map)[key][i_val] = val;
            }
            value_changed = value_changed || changed_tmp;
		}
	}
	std::cout << ANSI_COLOR_CYAN "========== update gain finished ===========" ANSI_COLOR_RESET << std::endl;
	if (value_changed) {
		control_hup_p->controller_p->update_gain_values();
		control_hup_p->controller_p->apply_gain_values();
	}
	control_hup_p->reset_controller(false, reset_nom);
	control_hup_p->updating_controller -= 1;
	std::cout << "updating_controller: " << control_hup_p->updating_controller <<std::endl;
}

#ifdef _WIN32
unsigned __stdcall RNB::serve_controller_server(void* control_hub_vp) {
#else
void* RNB::serve_controller_server(void* control_hub_vp) {
#endif
	ControlHub* control_hup_p = (ControlHub*) control_hub_vp;
	control_hup_p->stop_thread = false;
	control_hup_p->thread_stopped = false;

	int server_sockfd, client_sockfd;
	int client_len, n;
	char buf[MAXBUF];
	char path[MAXBUF];
	std::string html;
	void * controller_lib=NULL;

	// controller dropdown list)
	std::string control_type_name;
	std::string control_name_camel;
	switch(control_hup_p->mode){
	case ControllerInterface::JOINT_CONTROL:
		control_name_camel = "JointControl";
		control_type_name = "joint_control";
		break;
	case ControllerInterface::TASK_CONTROL:
		control_name_camel = "TaskControl";
		control_type_name = "task_control";
		break;
	}
	Dropdown dropdown_controller("/controller_list", control_type_name.c_str(), "Select controller: ");
	//update controller list
	update_controller_list(dropdown_controller, CONTROLLER_PATH);
	dropdown_controller.add_item(DEFAULT_CONTROLLER_NAME, DEFAULT_CONTROLLER_NAME);
	if (dropdown_controller.selected==""){
		dropdown_controller.select(DEFAULT_CONTROLLER_NAME);
	}

	// Parameter form
	MultipleInputForm param_setting("/param_setting");
	param_setting.clear();

	controller_lib=change_controller(
			control_hup_p, dropdown_controller, param_setting, controller_lib);

	// HTML Chart
	HtmlChart hchart(&(control_hup_p->data_logger), "/plot_step", "/pause_log", "/download_log");

#ifdef _WIN32
	SOCKADDR_IN clientaddr, serveraddr;

	WSADATA wsaData;

	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
		printf("WSAStartup() error!");

	memset(&serveraddr, 0, sizeof(serveraddr));
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
#else
	struct sockaddr_in clientaddr, serveraddr;
	memset(&serveraddr, 0, sizeof(serveraddr));
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
#endif
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(control_hup_p->ui_port);

	if ((server_sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		printf(ANSI_COLOR_RED "socket error\n" ANSI_COLOR_RESET);
		perror("socket error\n");
		throw "socket error\n";
	}


#ifdef _WIN32
	ULONG l;
	l = 1;
#else
	struct linger l;
	l.l_onoff = 1;
	l.l_linger = 0;
#endif
    setsockopt(server_sockfd,                // SOCKET
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
    setsockopt(server_sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv_w, sizeof tv_w);
#else
	setsockopt(server_sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
#endif

	bind(server_sockfd, (struct sockaddr*) &serveraddr, sizeof(serveraddr));
	if (listen(server_sockfd, 5) < 0) {
		printf(ANSI_COLOR_RED "===============================================\n" ANSI_COLOR_RESET);
		printf(ANSI_COLOR_RED "========== Control UI Listen Error ============\n" ANSI_COLOR_RESET);
		printf(ANSI_COLOR_RED "===============================================\n" ANSI_COLOR_RESET);
		throw "Listen Error";
	}
	printf(ANSI_COLOR_CYAN "===============================================\n" ANSI_COLOR_RESET);
	printf(ANSI_COLOR_CYAN "============= Control UI listening ============\n" ANSI_COLOR_RESET);
	printf(ANSI_COLOR_CYAN "===============================================\n" ANSI_COLOR_RESET);

	std::string path_string = "";
	std::string html_content = "";


	fd_set read_fds, tmp_fds;
	FD_ZERO(&read_fds);
	FD_SET(server_sockfd, &read_fds);
	while (!control_hup_p->stop_thread) {
		tmp_fds = read_fds;
		n = select(server_sockfd + 1, &tmp_fds, NULL, NULL, &tv);
		if (!n) continue;
		client_len = sizeof(clientaddr);
		memset(buf, 0x00, MAXBUF);
		memset(path, 0, MAXBUF);
		html = "";

		client_sockfd = accept(server_sockfd, (struct sockaddr *) &clientaddr,
				reinterpret_cast<__socklen_t *>(&client_len));
		if ((n = __read_sock(client_sockfd, buf, MAXBUF)) <= 0) {
			__close_sock(client_sockfd);
			continue;
		}

		printf(ANSI_COLOR_CYAN "===============================================\n" ANSI_COLOR_RESET);
		printf(ANSI_COLOR_CYAN "Control UI Client accept : %s\n" ANSI_COLOR_RESET, inet_ntoa(clientaddr.sin_addr));
		printf(ANSI_COLOR_CYAN "===============================================\n" ANSI_COLOR_RESET);

		// method
		for (int i = 0; i < strlen(buf); i++) {
			// printf("%d\n",i);
			if (buf[i] == 'G' && buf[i + 1] == 'E' && buf[i + 2] == 'T'
					&& buf[i + 3] == ' ') {
				for (int j = 0; buf[i + 4 + j] != ' '; j++) {
					path[j] = buf[i + 4 + j];
				}
				break;
			} else if (buf[i] == 'P' && buf[i + 1] == 'O' && buf[i + 2] == 'S'
					&& buf[i + 3] == 'T' && buf[i + 4] == ' ') {
				for (int j = 0; buf[i + 4 + j] != ' '; j++) {
					path[j] = buf[i + 4 + j];
				}
				break;
			}
		}

		// printf("request: %s \n", path);
		path_string = path;

		if (path_string.compare("/" CHART_MIN_JS) == 0) {
			// HTTP
			if (__write_sock(client_sockfd, HTML_HEADER, strlen(HTML_HEADER)) <= 0) {
				printf(ANSI_COLOR_RED "write error\n"  ANSI_COLOR_RESET);
				perror("write error\n");
				__close_sock(client_sockfd);
				continue;
			}
			html = hchart.chart_js_str;
		} else if(hchart.update_down(path_string, html)){
			// HTTP
			if (__write_sock(client_sockfd, HTTP_FILE_HEADER, strlen(HTTP_FILE_HEADER)) <= 0) {
				printf(ANSI_COLOR_RED "write error\n" ANSI_COLOR_RESET);
				perror("write error\n");
				__close_sock(client_sockfd);
				continue;
			}
		} else {
			// HTTP
			if (__write_sock(client_sockfd, HTML_HEADER, strlen(HTML_HEADER)) <= 0) {
				printf(ANSI_COLOR_RED "write error\n" ANSI_COLOR_RESET);
				perror("write error\n");
				__close_sock(client_sockfd);
				continue;
			}

			if (path_string.rfind("/shutdown") == 0) {
				html = HTML_CONTENT_HEAD
						"<h1>404- Not Found</h1>\n"
						HTML_CONTENT_TAIL;
				control_hup_p->stop_thread = true;
			} else {
				update_controller_list(dropdown_controller, CONTROLLER_PATH);
				dropdown_controller.add_item(DEFAULT_CONTROLLER_NAME, DEFAULT_CONTROLLER_NAME);
				std::string controller_bak = dropdown_controller.selected;
				bool is_controller_changed = dropdown_controller.update(
						path_string.c_str());
				if (is_controller_changed
						&& (controller_bak != dropdown_controller.selected)) {
					controller_lib=change_controller(
							control_hup_p, dropdown_controller, param_setting, controller_lib);
				}
				bool is_param_updated = param_setting.update(path_string.c_str());
				if (is_param_updated) {
					apply_gains(control_hup_p, param_setting, false);
				}
				html_content = HTML_CONTENT_HEAD;
				html_content += ("<h2> " + control_name_camel + " Hub </h2>\n");
				html_content += dropdown_controller.get_html();
				html_content += param_setting.get_html();
				html_content += hchart.get_html();
				html_content += "<a href=\"/shutdown\"> Shutdown Web UI Thread </a>\n";
				html_content += HTML_CONTENT_TAIL;
				html = html_content.c_str();

			}
		}

		if (__write_sock(client_sockfd, html.c_str(), html.size()) <= 0) {
			perror("write error:");
			__close_sock(client_sockfd);
			continue;
		}
		__close_sock(client_sockfd);
	}
	__close_sock(server_sockfd);
	std::cout << ANSI_COLOR_CYAN "======================= Control UI server closed ============================" ANSI_COLOR_RESET <<std::endl;

#ifdef _WIN32
	WSACleanup();
#endif

	if (control_hup_p->controller_p){
		std::cout << ANSI_COLOR_RED "========= unload controller =========" ANSI_COLOR_RESET
			<< std::endl;
		control_hup_p->controller_p.reset(); // resetting shared pointer to NULL. NOT reset_controller!!
	}

	if (controller_lib != NULL) {
#ifdef _WIN32
		FreeLibrary((HINSTANCE)controller_lib);
#else
		dlclose(controller_lib);
#endif
	}
	control_hup_p->thread_stopped = true;
}
