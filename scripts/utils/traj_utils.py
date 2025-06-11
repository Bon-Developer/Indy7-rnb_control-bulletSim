import numpy as np
from copy import deepcopy
from .joint_utils import *

def sign_positive_bias(Q):
    signQ = np.sign(Q)
    return signQ.astype(np.int) + (signQ==0).astype(np.int)

##
# @brief calculate cubic interpolation coefficients (x_t = a*t^3+b*t^2+c*t+d)
# @param dt  step time between q0, q1, q2
# @param v0 initial velocity at q0
# @return a,b,c,d in (x_t = a*t^3+b*t^2+c*t+d)
def calc_cubic_coeffs(dt, v0, q0, q1, q2):
    Tmat = np.array([[dt ** 3, dt ** 2], [8 * dt ** 3, 4 * dt ** 2]])  # (2,2)
    VQ = np.array([q1 - v0 * dt - q0, q2 - 2 * v0 * dt - q0])  # (2,DIM)
    if len(VQ.shape) == 1:
        VQ = VQ[:, np.newaxis]
    # Tmat*[a,b]'=VQ
    a, b = np.matmul(np.linalg.inv(Tmat), VQ)
    return a, b, v0, q0


##
# @brief calculate cubic interpolated trajectory (x_t = a*t^3+b*t^2+c*t+d)
def calc_cubic_traj(t, a, b, c, d):
    if not isinstance(t, float):
        t = t[:,np.newaxis]
    return a * t ** 3 + b * t ** 2 + c * t + d


##
# @brief calculate cubic interpolated trajectory velocity (v_t = 3*a*t^2+2*b*t+c)
def calc_cubic_vel(t, a, b, c):
    return 3 * a * t ** 2 + 2 * b * t + c


##
# @brief waypoint period to observe acc&vel limits with cubic interpolation
# @remark   Check velocity and acceleration limits by |3at^2+2bt+c|<Vmax, |6at+2b|<Amax. \n
#           Applying time scale r to at^3+bt^2+ct+d=Q makes ar^3t^3+br^2t^2+crt+d=Q,
#           makeing a_new = ar^3, b_new=br^2, c_new = cr. \n
#           But this implementation is only an estimation and does not give an optimal answer,
#           as c_new = c = v0 is a given constant value.
# @dt_step minimal timestep
# @return dt, a, b, c, d
def calc_cubic_coeffs_safe(dt_step, v0, q0, q1, q2, vel_lim, acc_lim, dt_init=None):
    all_pass = False
    dt_cur = np.max(np.abs(q1 - q0) / vel_lim) if dt_init is None else dt_init
    if dt_cur < dt_step:
        dt_cur = dt_step

    while not all_pass:
        a, b, c, d = calc_cubic_coeffs(dt_cur, v0, q0, q1, q2)
        a6T, b2 = 6 * a * dt_cur, 2 * b
        a6T_b2 = a6T + b2
        a3T2, b2T = 3 * a * dt_cur ** 2, 2 * b * dt_cur
        a3T2_b2T_c = a3T2 + b2T + c

        if np.max(np.abs(b2) / acc_lim) > 1:                # at t=0, |2b|<Amax
            ratio = np.sqrt(np.min(acc_lim/np.abs(b2))) * 0.99  # give 1% margin
        elif np.max(np.abs(a6T_b2) / acc_lim) > 1:            # at t=T, |6aT+2b|<Amax
            dir_c2 = sign_positive_bias(a6T_b2)
            roots = [np.roots([c1, c2, 0, c4]) for c1, c2, c4 in zip(a6T, b2,  -dir_c2*acc_lim)]
            ratio = np.min(
                [np.min([np.real(rt) for rt in root_single if np.imag(rt) < 1e-6 and np.real(rt) > 0]) for
                 root_single in roots]) * 0.99          # give 1% margin
        elif np.max(np.abs(a3T2_b2T_c) / vel_lim) > 1:            # at t=T, |3aT^2+2bT+c|<Vmax
            dir_c3 = sign_positive_bias(a3T2_b2T_c)
            roots = [np.roots([c1, c2, c3, c4]) for c1, c2, c3, c4 in zip(a3T2, b2T, c,  -dir_c3*vel_lim)]
            ratio = np.min(
                [np.min([np.real(rt) for rt in root_single if np.imag(rt) < 1e-6 and np.real(rt) > 0]) for
                 root_single in roots]) * 0.99          # give 1% margin
        else:
            all_pass = True
            continue

        dt_cur = dt_cur/ratio
        dt_cur = np.ceil(dt_cur / dt_step) * dt_step
    return dt_cur, a, b, c, d


##
# @brief get safe cubic interpolated trajectories
def get_safe_cubics(dt_step, trajectory, vel_lim, acc_lim, slow_start=False):
    T_list = []
    Q_list = []
    V_list = []
    v0 = np.zeros_like(trajectory[0])
    traj_len = len(trajectory)
    for i in range(traj_len):
        q0 = trajectory[i]
        q1 = trajectory[i + 1] if i + 1 < traj_len else trajectory[-1]
        q2 = trajectory[i + 2] if i + 2 < traj_len else trajectory[-1]
        if slow_start:
            dt, a, b, c, d = calc_cubic_coeffs_safe(dt_step, v0, q0, q1, q2, # first 1/4 is slow-start
                                                    vel_lim=vel_lim*max(0.2, min(1, (float(i+1)/traj_len)**2*4)),
                                                    acc_lim=acc_lim*max(0.2, min(1, (float(i+1)/traj_len)**2*4)))
        else:
            dt, a, b, c, d = calc_cubic_coeffs_safe(dt_step, v0, q0, q1, q2, vel_lim=vel_lim, acc_lim=acc_lim)
        v0 = calc_cubic_vel(dt, a, b, c)
        T_list.append(dt)
        Q_list.append(q0)
        V_list.append(v0)
    return T_list, Q_list, V_list


##
# @brief get full cubic trajectory for given waypoints
# @param dt_step    control time step
# @param T_list     List of time for each waypoint to the next one, last has no meaning-give 0. \n
#                   Each T in T list should be integer multiplication of dt_step
# @param Q_list     List of waypoint values (vectors)
def get_traj_all(dt_step, T_list, Q_list):
    t_all = []
    traj_all = []
    traj_len = len(Q_list)
    t0 = 0
    v0 = np.zeros_like(Q_list[0])
    for i in range(traj_len):
        T = T_list[i]
        if T <= dt_step:
            continue
        q0 = Q_list[i]
        q1 = Q_list[i + 1] if i + 1 < traj_len else Q_list[-1]
        q2 = Q_list[i + 2] if i + 2 < traj_len else Q_list[-1]
        a, b, c, d = calc_cubic_coeffs(T, v0, q0, q1, q2)
        for t in np.arange(0, T-dt_step/2, dt_step):
            t_all.append(t0 + t)
            traj_all.append(calc_cubic_traj(t, a, b, c, d))
        v0 = calc_cubic_vel(T, a, b, c)
        t0 += T
    return t_all, traj_all


##
# @brief get full cubic trajectory for given waypoint trajectory
# @remark terminal deceleration considered
def calc_safe_cubic_traj(dt_step, trajectory, vel_lim, acc_lim, slow_start=True):
    print("WARNING: This function is deprecated. USE calc_safe_trajectory")
    # calculate trajectory in forward and backward direction to consider deceleration
    T_list, Q_list, _ = get_safe_cubics(dt_step, trajectory, vel_lim=vel_lim, acc_lim=acc_lim, slow_start=slow_start)
    Trev_list, Qrev_list, _ = get_safe_cubics(dt_step, np.array(list(reversed(trajectory))), vel_lim=vel_lim,
                                              acc_lim=acc_lim, slow_start=slow_start)

    # weighting values to mix waypoint times, in S curves
    Tlen = len(T_list)-1
    alphaT = (np.arange(Tlen).astype(np.float) / (Tlen - 1))
    alphaT = (np.cos((alphaT + 1) * np.pi) + 1) / 2     # backward weights
    betaT = 1 - alphaT                                  # forward weights
    # mix waypoint times with weights
    T_list = list(((betaT*np.array(T_list[:-1])
                    + alphaT*np.array(list(reversed(Trev_list[:-1]))))
                   / dt_step).astype(np.int)*dt_step) # make sure each T is integer multiplication of dt_step
    T_list, Trev_list = T_list + [0], list(reversed(T_list)) + [0]
    # T_accum = [np.sum(T_list[:i]) for i in range(len(T_list))]

    # re-calculate trajectory with mixed waypoint times
    t_all, traj_all = get_traj_all(dt_step, T_list, Q_list)
    trev_all, trajrev_all = get_traj_all(dt_step, Trev_list, Qrev_list)
    traj_all, trajrev_all = np.array(traj_all), np.array(list(reversed(trajrev_all)))

    # weighting values to mix waypoint times, in S curves
    traj_len = len(traj_all)
    alpha = (np.arange(traj_len).astype(np.float) / (traj_len - 1))[:, np.newaxis]
    alpha = (np.cos((alpha + 1) * np.pi) + 1) / 2
    beta = 1 - alpha

    # mix forward and backward trajectory with S curve weights
    traj_tot = beta * traj_all + alpha * trajrev_all
    return traj_tot


##
# @brief calculate safe waypoint-time list for a list of waypoints, by simply iterating velocity & acceleration limits
# @param trajectory list of waypoints
# @param vel_lims   velocity limits, either in vector or scalar
# @param acc_lims   acceleration limits, either in vector or scalar
def calc_T_list_simple(trajectory, vel_lims, acc_lims,
                        vel_margin_ratio=0.1, acc_margin_ratio=0.1,
                        upper_bound_ratio=1.2, low_bound_ratio=0.5):
    dq_list = np.subtract(trajectory[1:], trajectory[:-1])
    idx_skip = np.where(np.linalg.norm(dq_list, axis=-1)<1e-3)[0]
    idx_valid = np.where(np.linalg.norm(dq_list, axis=-1)>=1e-3)[0]
    dq_list = dq_list[idx_valid]
    dT_list = np.max(np.abs(dq_list)/vel_lims, axis=-1)
    dT_list_mid_pre = None
    alpha = 0.5
    for _ in range(1000):
        dT_list_pre = dT_list # between q, N-1
        vel_list = dq_list/dT_list[:,np.newaxis] # between q, N-1
        vel_list = np.pad(vel_list, ((1,1),(0,0)), 'constant', constant_values=0) # between q and expand, N+1
        dT_list = np.pad(dT_list, (1,1), 'constant', constant_values=0) # between q and expand, N+1
        dv_list = vel_list[1:]- vel_list[:-1] # on q, N
        vel_mid_list = np.abs((vel_list[1:] + vel_list[:-1])/2) # on q, N
        vel_ratio_mid = np.max(vel_mid_list/(vel_lims*(1+vel_margin_ratio)), axis=-1) # on q, N
        dT_list_mid = (dT_list[:-1]+dT_list[1:])/2 # on q, N
        acc_list = np.abs(dv_list) / dT_list_mid[:,np.newaxis] # on q, N
        acc_ratio = np.max(acc_list/(acc_lims*(1+acc_margin_ratio)), axis=-1) # on q, N
        vel_pass = vel_ratio_mid < upper_bound_ratio
        acc_pass = acc_ratio < upper_bound_ratio
        low_bound_pass = np.logical_or(vel_ratio_mid > low_bound_ratio, acc_ratio > low_bound_ratio)
        # calc_T_list_simple.pass_list= [vel_pass, acc_pass, low_bound_pass]
        # calc_T_list_simple.ratio_list= [vel_ratio_mid, acc_ratio]
        if  np.all(vel_pass) and np.all(acc_pass) and np.all(low_bound_pass):
            dT_list = dT_list[1:-1]
            break
        adjust_ratio = np.max([
                                acc_ratio*np.logical_not(acc_pass),
                                vel_ratio_mid*np.logical_not(vel_pass),
                                np.maximum(acc_ratio, vel_ratio_mid)*np.logical_not(low_bound_pass)], axis=0) \
                                +np.all([acc_pass, vel_pass, low_bound_pass], axis=0) # on q, N
        if dT_list_mid_pre is None:
            dT_list_mid_new = dT_list_mid*adjust_ratio # on q, N
        else:
            dT_list_mid_new = alpha*dT_list_mid*adjust_ratio + (1-alpha)*dT_list_mid_pre # on q, N
        dT_list_mid_pre = deepcopy(dT_list_mid_new)
        dT_list_mid_new[0] *= 2   # double first and last acc time as they are half-step
        dT_list_mid_new[-1] *= 2   # double first and last acc time as they are half-step
        dT_list = (dT_list_mid_new[:-1]+dT_list_mid_new[1:])/2 # between q, N-1
    T_list = np.pad(dT_list, (0,1), 'constant', constant_values=0)
    for idx in idx_skip:
        T_list = np.insert(T_list, idx, 0)
    return T_list


##
# @brief get full cubic trajectory for given waypoint trajectory
# @remark terminal deceleration considered
def calc_safe_trajectory(dt_step, trajectory, vel_lims, acc_lims):
    # calculate waypoint times
    T_list = calc_T_list_simple(trajectory, vel_lims, acc_lims)

    # round waypoint times with dt_step
    T_list_new = []
    Q_list = []
    T_stack = 0
    for Q, T in zip(trajectory, T_list):
        T_stack += T
        if T_stack > dt_step:
            T_list_new.append(np.ceil(T_stack / dt_step) * dt_step)
            Q_list.append(Q)
            T_stack = 0
    if np.any(Q != Q_list[-1]):
        T_list_new.append(np.ceil(T_stack / dt_step) * dt_step)
        Q_list.append(Q)
    T_list = np.array(T_list_new)
    Q_list = np.array(Q_list)

    # prepare backward waypoints and times
    Qrev_list = np.array(list(reversed(Q_list)))
    Trev_list = list(reversed(T_list[:-1])) + [0]

    # re-calculate trajectory with mixed waypoint times
    t_all, traj_all = get_traj_all(dt_step, T_list, Q_list)
    trev_all, trajrev_all = get_traj_all(dt_step, Trev_list, Qrev_list)
    traj_all, trajrev_all = np.array(traj_all), np.array(list(reversed(trajrev_all)))

    # weighting values to mix waypoint times, in S curves
    traj_len = len(traj_all)
    alpha = (np.arange(traj_len).astype(np.float) / (traj_len - 1))[:, np.newaxis]
    alpha = (np.cos((alpha + 1) * np.pi) + 1) / 2
    beta = 1 - alpha

    # mix forward and backward trajectory with S curve weights
    traj_tot = beta * traj_all + alpha * trajrev_all
    return t_all, traj_tot

##
# @brief calculate sweep trajectory
# @param gtem GeometryItem
# @param dP_tar target delta pos in ref_link
def get_sweep_traj(urdf_content, joint_names, tip_link, dP_tar, Q0, DP=0.01, ERROR_CUT=0.01, SINGULARITY_CUT = 0.01,
                   VERBOSE=False, ref_link=None):
    joint_limits = [(urdf_content.joint_map[jname].limit.lower,
                     urdf_content.joint_map[jname].limit.upper) for jname in joint_names]
    Q = Q0
    if ref_link is None:
        ref_link = get_link_names(urdf_content)[0]
    T0 = Tnew = get_tf(tip_link, list2dict(Q0, joint_names), urdf_content, from_link=ref_link)
    P0 = T0[:3, 3]
    Traj = []
    dPnorm = np.linalg.norm(dP_tar)
    DIR = np.concatenate([np.divide(dP_tar, dPnorm), [0, 0, 0]])
    P0 = np.concatenate([P0, [0, 0, 0]])
    reason = "end"
    singularity = False
    for dP_cur in np.arange(0, dPnorm + DP / 2, DP):
        Jac = get_jacobian(tip_link, urdf_content, Q, joint_names, ref_link=ref_link)
        if np.min(np.abs(np.real(np.linalg.svd(Jac)[1]))) <= SINGULARITY_CUT:
            singularity = True
            reason = "singular"
            break
        Jinv = np.linalg.pinv(Jac)
        Ptar = np.multiply(DIR, dP_cur) + P0
        Pcur = np.concatenate([Tnew[:3, 3], [0, 0, 0]])
        dQ = np.matmul(Jinv, Ptar - Pcur)
        Q = Q + dQ
        
        dlim = np.subtract(joint_limits, Q[:, np.newaxis])[np.where(np.sum(np.abs(Jac), axis=0) > 1e-6)[0], :]
        if np.any(dlim[:, 0] > 0):
            reason = "joint min"
            break
        if np.any(dlim[:, 1] < 0):
            reason = "joint max"
            break
        Tnew = get_tf(tip_link, list2dict(Q, joint_names), urdf_content, from_link=ref_link)
        dPcalc = Tnew[:3, 3] - T0[:3, 3]
        dRcalc = Rotation.from_matrix(np.matmul(Tnew[:3, :3], T0[:3, :3].transpose())).as_rotvec()
        if np.abs(np.linalg.norm(dPcalc) - np.dot(dPcalc, DIR[:3])) > ERROR_CUT:
            reason = "error off"
            break
        Traj.append(Q)
    if VERBOSE:
        print(reason)
    return np.array(Traj), reason=="end"