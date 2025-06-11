from ..utils import *
from ..traj_utils import *
import abc


DEFAULT_TRAJ_PORT = 9980
DEFAULT_TRAJ_FREQUENCY = 50

##
# @class TrajectoryClient
# @brief TCP/IP client for trajectory Interface
# @remark use move_joint_s_curve to send off-line trajectory
#         use move_joint_s_curve_online to test online trajectory following.
class TrajectoryClient(object):
    MAX_INIT_ERROR = 8e-2

    def __init__(self, server_ip, server_port=DEFAULT_TRAJ_PORT, traj_freq=DEFAULT_TRAJ_FREQUENCY):
        self.server_ip, self.server_port, self.traj_freq = \
            server_ip, server_port, traj_freq
        ##
        # @brief periodic timer with the given traj_freq
        self.periodic = PeriodicTimer(1.0/traj_freq)
        ##
        # @brief periodic timer 4 times faster than the given traj_freq, to give margins
        self.periodic_x4 = PeriodicTimer(1.0/traj_freq/4)
        self.qcount = 0

    def reset(self):
        self.qcount = self.get_qcount()
        reset_dict = {'reset': 1, 'period_s': 1.0 / self.traj_freq, }
        return send_recv(reset_dict, self.server_ip, self.server_port)

    def get_qcount(self):
        return send_recv({'qcount': 0}, self.server_ip, self.server_port)['qcount']

    def get_qcur(self):
        self.qcur = send_recv({'getq': 0}, self.server_ip, self.server_port)['qval']
        return self.qcur

    def send_qval(self, qval):
        return send_recv({'qval': qval}, self.server_ip, self.server_port)

    def start_tracking(self):
        return send_recv({'follow': 1}, self.server_ip, self.server_port)

    def pause_tracking(self):
        return send_recv({'pause': 1}, self.server_ip, self.server_port)

    def resume_tracking(self):
        return send_recv({'resume': 1}, self.server_ip, self.server_port)

    def stop_tracking(self):
        res = send_recv({'stop': 1}, self.server_ip, self.server_port)
        self.wait_queue_empty(max_dur=1.0/self.traj_freq*10)
        return res

    def terminate_loop(self):
        return send_recv({'terminate': 1}, self.server_ip, self.server_port)

    ##
    # @brief Wait until the queue on the server is empty. This also means the trajectory motion is finished.
    def wait_queue_empty(self, max_dur=1000):
        time_start = time.time()
        while self.get_qcount()>0:
            self.periodic.wait()
            if (time.time() - time_start) > max_dur:
                TextColors.RED.println("[WARN] ROBOT MOTION TIMEOUT")
                break
        time.sleep(1.0 / self.traj_freq * 2)

    ##
    # @brief    Send target pose to the server and store the queue count.
    # @remark If online mode, this will not send value but return False and sleep for a quarter period when trajectory server is not ready
    # @param online If this flag is set True, it will wait the queue on the server to sync the motion.
    def push_Q(self, Q, online=False):
        if online:
            if self.qcount >= 3:
                self.periodic_x4.wait()
            if self.qcount > 3:
                self.qcount = self.get_qcount()
                sent = False
                return sent
        self.qcount = self.send_qval(Q)['qcount']
        sent = True
        return sent

    ##
    # @brief send off-line s-surve trajectory to move joint to target position
    # @param qtar           target joint configuration
    # @param N_div          the number of divided steps (default=100)
    # @param wait_finish    send trajectory off-line but wait until finish (default=100)
    # @param start_tracking to reset trajectory and start tracking
    # @param auto_stop      auto-stop trajectory-following after finishing the motion
    def move_joint_s_curve(self, qtar, q0=None, N_div=100, wait_finish=True, start_tracking=True, auto_stop=True):
        qcur = np.array(self.get_qcur()) if q0 is None else q0
        DQ = qtar - qcur
        if start_tracking:
            self.reset()
        self.pause_tracking()
        i_step = 0
        while i_step < N_div+1:
            Q = qcur + DQ * (np.sin(np.pi * (float(i_step) / N_div - 0.5)) + 1) / 2
            i_step += self.push_Q(Q)
        if start_tracking:
            self.start_tracking()
        else:
            self.resume_tracking()
            
        if wait_finish:
            self.wait_queue_empty()
            if auto_stop:
                self.stop_tracking()
        else:
            assert not auto_stop, "Cannot auto-stop trajectory following not waiting end of the motion"

    ##
    # @brief    send s-surve trajectory on-line to move joint to target position.
    #           To test on-line trajectory following for adaptive motion
    # @param qtar         target joint configuration
    # @param N_div          the number of divided steps (default=100)
    # @param start_tracking to reset trajectory and start tracking
    # @param auto_stop      auto-stop trajectory-following after finishing the motion
    def move_joint_s_curve_online(self, qtar, q0=None, N_div=100, start_tracking=True, auto_stop=True):
        qcur = np.array(self.get_qcur()) if q0 is None else q0
        DQ = qtar - qcur

        if start_tracking:
            self.reset()
            self.start_tracking()
        i_step = 0
        while i_step < N_div+1:
            Q = qcur + DQ * (np.sin(np.pi * (float(i_step) / N_div - 0.5)) + 1) / 2
            i_step += self.push_Q(Q, online=True) # increase index if push_Q returns True
        if auto_stop:
            self.wait_queue_empty()
            self.stop_tracking()

    ##
    # @param trajectory radian
    # @param vel_lims radian/s, scalar or vector
    # @param acc_lims radian/s2, scalar or vector
    # @return interpolated trajecotry, expected motion time
    def move_joint_wp(self, trajectory, vel_lims, acc_lims, auto_stop=True):
        trajectory = np.concatenate([[self.get_qcur()], trajectory])
        t_all, traj_tot = calc_safe_trajectory(1.0/self.traj_freq, trajectory, vel_lims=vel_lims, acc_lims=acc_lims)
        self.pause_tracking()
        for Q in traj_tot:
            self.push_Q(Q)
        self.start_tracking()
        self.wait_queue_empty()
        if auto_stop:
            self.stop_tracking()
        return traj_tot, float(len(traj_tot))/self.traj_freq

    ##
    # @brief Move joints to Q using the most guaranteed method for each robot.
    # @remark Robot-specific implementation is required.
    # @param Q radian
    def joint_move_make_sure(self, Q, auto_stop=True, ref_speed=np.pi/6): # default reference speed: 30deg/s
        Qcur = self.get_qcur()
        link_scale = np.arange(len(Qcur), 0, -1, dtype=float)
        Qdiff = np.max(np.abs(np.subtract(Q, Qcur)*link_scale))
        move_time = Qdiff / ref_speed
        N_div = int(move_time * self.traj_freq)
        if N_div < 5:
            N_div = 5
        self.move_joint_s_curve(Q, N_div=N_div, auto_stop=auto_stop)

    ##
    # @brief Execute grasping
    # @remark Robot-specific implementation is required.
    # @param grasp True: grasp / False: release
    @abc.abstractmethod
    def grasp(self, grasp):
        TextColors.YELLOW.println("Robot-specific implementation is required for grasp function")

    ##
    # @brief move joint with waypoints, one-by-one
    # @param trajectory numpy array (trajectory length, joint num)
    def move_joint_traj(self, trajectory, auto_stop=True, wait_motion=True):
        Q_init = trajectory[0]
        Q_last = trajectory[-1]

        qcur = self.get_qcur()
        qerr = np.max(np.abs(np.subtract(qcur, trajectory[0])))
        if qerr >self.MAX_INIT_ERROR:
            raise(RuntimeError(("move_joint_traj: error too much: {} / {} \n{} / \n{}".format(
                np.round(np.rad2deg(qerr), 1), np.round(np.rad2deg(self.MAX_INIT_ERROR), 1),
                list(np.round(np.rad2deg(qcur), 1)), list(np.round(np.rad2deg(Q_init), 1))))))

        self.pause_tracking()
        for Q in trajectory:
            self.push_Q(Q)
            
        self.start_tracking()

        if wait_motion:
            self.wait_queue_empty()

            if auto_stop:
                self.stop_tracking()
                time.sleep(0.2)
