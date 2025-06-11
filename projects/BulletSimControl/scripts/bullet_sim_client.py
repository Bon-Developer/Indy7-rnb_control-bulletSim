import sys
import os

ROOT_NAME = "rnb-control"
RNB_CONTROL_DIR = __file__[:__file__.find(ROOT_NAME)+len(ROOT_NAME)]
root_script_path = os.path.join(RNB_CONTROL_DIR, "scripts")
sys.path.append(root_script_path)
from utils.rotation_utils import *
from utils.utils import *
import time
from collections import namedtuple
from enum import Enum

import pybullet as pb

physics_client = -1
plugin_id = -1
NOMINAL_COLOR = [1,0,0,0.3]

val_names = ['q', 'qdot', 'qddot', 'q_n', 'qdot_n', 'p', 'p_n', 
             'M', 'C', 'M_n', 'C_n', 'tauGrav', 'tauExt', 
             'J', 'Jdot', 'J_n', 'Jdot_n', 'FText', "torque_out", "qddot_n"]
DynState = namedtuple("DynState", val_names)

def init_bullet_sim(gravity_vec=(0,0,-9.80665), time_step=1.0/1000,
                    plugin_name="ControlHubPlugin"):
    global physics_client, plugin_id
    assert physics_client<0, "Already connected to bullet server - client: {}".format(physics_client)
    assert plugin_id<0, "Already loaded ControlHubPlugin - id: {}".format(plugin_id)
    physics_client = pb.connect(pb.SHARED_MEMORY)
    pb.setGravity(*gravity_vec)
    pb.setTimeStep(time_step)
    
    ## load ControlHubPlugin
    plugin_id = pb.loadPlugin(plugin_name)
    assert plugin_id >=0, "Failed loading ControlHubPlugin"

class ControlMode(Enum):
    JOINT_CONTROL=0
    TASK_CONTROL=1    ## Not implemented yet
    RESET=2
    
class BulletControlHubClient:
    def __init__(self, urdf_file, xyz=[0,0,0], quat=[0,0,0,1], use_nominal=True, 
                 flags=pb.URDF_USE_INERTIA_FROM_FILE, **kwargs):
        self.robot_id = pb.loadURDF(urdf_file, xyz, quat,
                                   useMaximalCoordinates=False, flags=flags, **kwargs)
        self.joint_num = pb.getNumJoints(self.robot_id)
        self.TIP_LINK = self.joint_num - 1
        if use_nominal:
            self.nom_id = pb.loadURDF(urdf_file, xyz, quat,
                                         useMaximalCoordinates=False)
            for i in range(-1, self.joint_num):
                pb.setCollisionFilterGroupMask(self.nom_id, i, 0, 0)
            for i in range(-1, self.joint_num):
                pb.changeVisualShape(self.nom_id, i, rgbaColor=NOMINAL_COLOR)
        else:
            self.nom_id = -1
        self.DT = pb.getPhysicsEngineParameters()['fixedTimeStep']
        
        len_j = self.joint_num
        ## disable default control
        pb.setJointMotorControlArray(self.robot_id, range(len_j), 
                                    pb.POSITION_CONTROL, forces=[0]*len_j)
        pb.setJointMotorControlArray(self.robot_id, range(len_j), 
                                    pb.VELOCITY_CONTROL, forces=[0]*len_j)
        if use_nominal:
            pb.setJointMotorControlArray(self.nom_id, range(len_j), 
                                        pb.POSITION_CONTROL, forces=[0]*len_j)
            pb.setJointMotorControlArray(self.nom_id, range(len_j), 
                                        pb.VELOCITY_CONTROL, forces=[0]*len_j)
        
        ## enable last joint FT sensor
        pb.enableJointForceTorqueSensor(self.robot_id, len_j-1)
        
        
        self.controller_ready = False
        ## initialize state values
        self.initialize_state()
        
    ##
    # @brief prepare control hub
    # @param ui_port default is 9990 defined in control_hub.h 
    # @param traj_port default is 9980 defined in trajectory_interface.h 
    def set_control_hub(self, mode=ControlMode.JOINT_CONTROL, ui_port=0, traj_port=0):
        self.ui_port, self.traj_port = ui_port, traj_port
        
        res = pb.executePluginCommand(plugin_id, "", 
                                     [mode.value, 
                                      self.robot_id, self.nom_id, 
                                      self.ui_port, self.traj_port], 
                                     [])
        time.sleep(0.1)
        pb.stepSimulation()
        pb.setRealTimeSimulation(1) # ON
        time.sleep(0.1)
        pb.setRealTimeSimulation(0) # OFF
        assert res >=0, "Failed Initializaing ControlHubPlugin"
        self.controller_ready = True
        
    def reset_control(self):
        res = pb.executePluginCommand(plugin_id, "", 
                                     [2, self.robot_id], 
                                     [])
        
    def unload(self):
        self.controller_ready = False
        if self.robot_id >= 0:
            res = pb.executePluginCommand(plugin_id, "", 
                                         [3, 
                                          self.robot_id], 
                                         [])
            assert res >=0, "Failed unloding ControlHubPlugin for {}".format(self.robot_id)
            pb.removeBody(self.robot_id)
        if self.nom_id >= 0:
            pb.removeBody(self.nom_id)

    ##
    # @brief initialize controller state
    def initialize_state(self, Q=None,
                         use_real_kinematics=True, use_real_dynamics=True,
                         use_nominal_kinematics=True, use_nominal_dynamics=True):
        self.reset_cmd = True
        self.use_real_kinematics = use_real_kinematics
        self.use_real_dynamics = use_real_dynamics
        self.use_nominal_kinematics = use_nominal_kinematics
        self.use_nominal_dynamics = use_nominal_dynamics

        if Q is None:
            Q = np.zeros(self.joint_num)
        else:
            Q = np.copy(Q)
        for i in range(self.joint_num):
            pb.resetJointState(self.robot_id, i, Q[i])
            if self.nom_id>=0:
                pb.resetJointState(self.nom_id, i, Q[i])
        
        self.q = np.copy(Q)
        self.qdot = np.zeros(self.joint_num)
        self.qddot = np.zeros(self.joint_num)
        self.q_n = np.copy(Q)
        self.qdot_n = np.zeros(self.joint_num)
        self.qddot_n = np.zeros(self.joint_num)
        self.J = np.zeros((6, self.joint_num))
        self.J_n = np.zeros((6, self.joint_num))
        self.torque_out = np.zeros(self.joint_num)
        self.zeroQ = np.zeros(self.joint_num)
        if self.controller_ready:
            self.reset_control()
        time.sleep(0.1)
        pb.stepSimulation()

    ##
    # @brief update control variables
    # @remark returns q, qdot, qddot,q_n, qdot_n, p, p_n, 
    #         M, C, M_n, C_n, tauGrav, tauExt, J, Jdot, J_n, Jdot_n, FText
    def update_state(self):
        # read sensor
        jstate_list = pb.getJointStates(self.robot_id, range(self.joint_num))
        for i_j, jstate in enumerate(jstate_list):
            _q, _qdot, _rft, _tau = jstate
            self.qddot[i_j] = (_qdot-self.qdot[i_j]) / self.DT
            self.qdot[i_j] = _qdot
            self.q[i_j] = _q
        self.FText = np.array(jstate[2], dtype=float)

        # handle reset case
        reset_seq = False
        if self.reset_cmd:
            self.reset_cmd = False
            reset_seq = True
            self.q_n[:] = self.q[:]
            self.qdot_n[:] = self.qdot[:]
            self.qddot_n = np.zeros(self.joint_num, dtype=float)

        # ee position
        link_com_p, link_com_r, loc_com_p, loc_com_r, link_frame_p, link_frame_r = \
            pb.getLinkState(self.robot_id, self.TIP_LINK, 0, 1)
        T_wp = T_xyzquat((link_frame_p, link_frame_r))
        self.p = np.concatenate([T_wp[:3,3], list(reversed(Rot2zyx(T_wp[:3,:3])))])

        # nominal ee position
        if self.nom_id>=0:
            for i in range(self.joint_num):
                pb.resetJointState(self.nom_id, i, self.q_n[i])
            link_com_p, link_com_r, loc_com_p, loc_com_r, link_frame_p, link_frame_r = \
                pb.getLinkState(self.nom_id, self.TIP_LINK, 0, 1)
            T_wp = T_xyzquat((link_frame_p, link_frame_r))
            self.p_n = np.concatenate([T_wp[:3,3], list(reversed(Rot2zyx(T_wp[:3,:3])))])
        else:
            self.p = np.copy(self.p_n)

        # get gravity
        self.tauGrav = pb.calculateInverseDynamics(self.robot_id, self.q.tolist(), self.zeroQ.tolist(), self.zeroQ.tolist())

        if self.use_real_kinematics:
            Jpos, Jrot = pb.calculateJacobian(self.robot_id, self.TIP_LINK, [0,0,0],
                                             self.q.tolist(), self.qdot.tolist(), self.qddot.tolist())
            J = np.concatenate([Jpos, Jrot])
            if reset_seq:
                self.Jdot = np.zeros_like(J, dtype=J.dtype)
            else:
                self.Jdot = (J-self.J)/self.DT
            self.J = J

        if self.use_nominal_kinematics:
            Jpos, Jrot = pb.calculateJacobian(self.robot_id, self.TIP_LINK, [0,0,0],
                                             self.q_n.tolist(), self.qdot_n.tolist(), self.qddot_n.tolist())
            J = np.concatenate([Jpos, Jrot])
            if reset_seq:
                self.Jdot_n = np.zeros_like(J, dtype=J.dtype)
            else:
                self.Jdot_n = (J-self.J_n)/self.DT
            self.J_n = J

        if self.use_real_dynamics:
            self.M = pb.calculateMassMatrix(self.robot_id, self.q.tolist())

            # Get real coriolis by tau(q, qdot) - tau(q), approximate to diagonal
            tau_m = pb.calculateInverseDynamics(
                self.robot_id, self.q.tolist(), self.qdot.tolist(), self.zeroQ.tolist())
            tau_m = np.subtract(tau_m, self.tauGrav)
            cvec = tau_m/(self.qdot+(np.sign(self.qdot)+(self.qdot==0))*1e-16)
            self.C = np.diag(cvec);

        if self.use_nominal_dynamics:
            self.M_n = pb.calculateMassMatrix(self.robot_id, self.q_n.tolist())

            # Get real coriolis by tau(q, qdot) - tau(q), approximate to diagonal
            tau_g_n = pb.calculateInverseDynamics(
                self.robot_id, self.q_n.tolist(), self.zeroQ.tolist(), self.zeroQ.tolist())
            tau_m = pb.calculateInverseDynamics(
                self.robot_id, self.q_n.tolist(), self.qdot_n.tolist(), self.zeroQ.tolist())
            tau_m = np.subtract(tau_m, tau_g_n)
            cvec = tau_m/(self.qdot_n+(np.sign(self.qdot_n)+(self.qdot_n==0))*1e-16)
            self.C_n = np.diag(cvec);

        # Get external torque as deviation from inverse dynamics torque - the motor is considered to be ideal in bullet.
        tau_m = pb.calculateInverseDynamics(
            self.robot_id, self.q.tolist(), self.qdot.tolist(), self.qddot.tolist())
        self.tauExt = self.torque_out - tau_m

        # Get end-effector force torque by deviation between joint FT value and model acc
        axis = np.array(pb.getJointInfo(self.robot_id, self.TIP_LINK)[13], dtype=float)
        tau_j = axis*self.tauExt[self.TIP_LINK]
        self.FText[3:] += tau_j
        return DynState(
                self.q, self.qdot, self.qddot,\
                self.q_n, self.qdot_n, self.p, self.p_n,\
                self.M, self.C, self.M_n, self.C_n, \
                self.tauGrav, self.tauExt, \
                self.J, self.Jdot, self.J_n, self.Jdot_n, \
                self.FText, 0, 0)
    
    def apply_torque(self, torque_out, qddot_n):
        torque_out = torque_out.tolist() \
                if isinstance(torque_out, np.ndarray) \
                else torque_out

        self.q_n += (self.DT*self.qdot_n) + (0.5*self.DT*self.DT*qddot_n)
        self.qdot_n += (self.DT*qddot_n)
        self.qddot_n = qddot_n
        self.torque_out = np.array(torque_out)
        pb.setJointMotorControlArray(self.robot_id, range(self.joint_num),
                                     pb.TORQUE_CONTROL, forces=torque_out)
            
import matplotlib.pyplot as plt
from copy import deepcopy

def diag_if_mat(val):
    if len(np.array(val).shape)==2:
        return np.diag(val)
    return val

def plot_values(pc, dyn_values, i_v):
    val_name = val_names[i_v]
    plt.plot([diag_if_mat(dyn_val[i_v]) for dyn_val in dyn_values])
    plt.title(val_name)
    plt.legend([i for i in range(pc.joint_num)])
    
##
# @brief example adding simple collision body
# @param geom_type    pybullet.GEOM_{TYPE}
# @param mass         mass of the geometry, mass=0 means it is fixed body
# @param halfExtents  dimensions for GEOM_BOX
# @param radius       radius of GEOM_CAPSULE, GEOM_CYLINDER, GEOM_SPHERE
# @param height       height of GEOM_CAPSULE, GEOM_CYLINDER
def add_simple_geom(geom_type, center, quat, mass=0, 
                    radius=0.5, height=1, halfExtents=[1,1,1],
                    rgbaColor=(1,1,0,1), collision=True, **kwargs):
    if collision:
        cid = pb.createCollisionShape(geom_type, radius=radius, height=height, 
                                      halfExtents=halfExtents,
                                      collisionFramePosition=(0,0,0), 
                                      collisionFrameOrientation=(0,0,0,1), 
                                      **kwargs)
    else:
        cid = -1

    vid = pb.createVisualShape(geom_type, radius=radius, length=height, 
                                  halfExtents=halfExtents,
                                  visualFramePosition=(0,0,0), 
                                  visualFrameOrientation=(0,0,0,1),
                                  rgbaColor=rgbaColor, 
                                  **kwargs)

    return pb.createMultiBody(baseMass=mass,
                              baseInertialFramePosition=[0, 0, 0],
                              baseInertialFrameOrientation=[0, 0, 0],
                              baseCollisionShapeIndex=cid,
                              baseVisualShapeIndex=vid,
                              basePosition=center,
                              baseOrientation=quat)

def show_axis(center, quat, radius=0.01, length=0.1):
    Tb = T_xyzquat((center, quat))
    Tx = SE3(Rot_axis(2, np.pi/2), (length/2,0,0))
    Ty = SE3(Rot_axis(1, np.pi/2), (0,length/2,0))
    Tz = SE3(Rot_axis(1, 0), (0,0,length/2))
    Tbx = np.matmul(Tb, Tx)
    Tby = np.matmul(Tb, Ty)
    Tbz = np.matmul(Tb, Tz)

    ctqtx = T2xyzquat(Tbx)
    ctqty = T2xyzquat(Tby)
    ctqtz = T2xyzquat(Tbz)


    xid = add_simple_geom(pb.GEOM_CYLINDER, ctqtx[0], ctqtx[1], 
                          mass=0, radius=radius, height=length,
                          rgbaColor=(1,0,0,0.5), collision=False)
    yid = add_simple_geom(pb.GEOM_CYLINDER, ctqty[0], ctqty[1], 
                          mass=0, radius=radius, height=length,
                          rgbaColor=(0,1,0,0.5), collision=False)
    zid = add_simple_geom(pb.GEOM_CYLINDER, ctqtz[0], ctqtz[1], 
                          mass=0, radius=radius, height=length,
                          rgbaColor=(0,0,1,0.5), collision=False)

    return xid, yid, zid


class RobotType(Enum):
    KUKA_IIWA=0
    INDY7=1
    PANDA=2
    
def get_urdf_path(rtype):
    if rtype==RobotType.KUKA_IIWA:
        return os.path.join(RNB_CONTROL_DIR, "urdf/kuka_iiwa/model.urdf")
    if rtype==RobotType.INDY7:
        return os.path.join(RNB_CONTROL_DIR, "urdf/indy7/indy7.urdf")
    if rtype==RobotType.PANDA:
        TextColors.RED.println("[WARN] Inertial information is not provided with Panda. Dynamics functions will cause errors.")
        return os.path.join(RNB_CONTROL_DIR, "urdf/panda/panda_bullet.urdf")

    

__rt_thread = None
__rt_stop = False

def __rt_thread_fun(realtime=True,step_num=None):
    global __rt_thread, __rt_stop
    if realtime:
        DT = pb.getPhysicsEngineParameters()["fixedTimeStep"]
        timer = StrictTimer(DT)
    i_step = 0
    while not __rt_stop:
        if realtime:
            timer.sleep()
        pb.stepSimulation()
        i_step += 1
        if step_num is not None and i_step>=step_num:
            break
    print("[INFO] simulation thread stopped")
    __rt_thread, __rt_stop = None, False
        
##
# @brief start simulation thread
# @remark pybullet.setRealTimeSimulation shows very unstable behavior. use this function instead.
def start_simulation_thread(realtime=True, step_num=None):
    global __rt_thread
    if __rt_thread is None:
        __rt_thread = Thread(target=__rt_thread_fun, args=(realtime,step_num))
        __rt_thread.daemon = True
        __rt_thread.start()
    else:
        TextColors.YELLOW.println("[INFO] simulation thread is already running")
        
def stop_simulation_thread():
    global __rt_stop
    __rt_stop = True
    
def is_sim_stopped():
    return __rt_thread is None