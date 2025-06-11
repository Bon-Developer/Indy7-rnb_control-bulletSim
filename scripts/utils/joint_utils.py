from .utils import *
from .rotation_utils import *

def get_link_names(urdf_content):
    return urdf_content.get_chain(root=urdf_content.links[0].name, tip=urdf_content.links[-1].name, links=True, joints=False)

def get_joint_names(urdf_content):
    return urdf_content.get_chain(root=urdf_content.links[0].name, tip=urdf_content.links[-1].name, links=False, joints=True, fixed=False)

def get_parent_joint(link_name, urdf_content):
    return urdf_content.parent_map[link_name][0]

def get_joint_tf(joint, joint_dict):
    if joint.type in ['revolute', 'continuous']:
        T_J = SE3(Rot_axis(np.where(joint.axis)[0] + 1, joint_dict[joint.name]), [0,0,0])
    elif joint.type == 'prismatic':
        T_J = SE3(np.identity(3), np.multiply(joint.axis, joint_dict[joint.name]))
    elif joint.type == 'fixed':
        T_J = np.identity(4)
    else:
        raise NotImplementedError("joint type {} not defined".format(joint_type))
    Toff = SE3(Rot_rpy(joint.origin.rpy), joint.origin.xyz)
    # T = np.matmul(Toff, np.matmul(T_J,T))
    return np.matmul(Toff,T_J)

def get_tf(to_link, joint_dict, urdf_content, from_link='base_link'):
    T = np.identity(4)
    link_cur = to_link
    link_root = urdf_content.get_root()
    while link_cur != from_link:
        if link_cur != link_root:
            pjname = get_parent_joint(link_cur, urdf_content)
            if pjname is None:
                break
            parent_joint = urdf_content.joint_map[pjname]
            Tj = get_joint_tf(parent_joint, joint_dict)
            T = np.matmul(Tj,T)
            link_cur = parent_joint.parent
        else:
            T_from_link = get_tf(from_link, joint_dict=joint_dict, urdf_content=urdf_content, from_link=link_root)
            T = np.matmul(SE3_inv(T_from_link), T)
            break
    return T

##
# @brief calculate jacobian for a geometry movement
# @param gtem   GeometryItem
# @param Q      current joint pose as array
# @param ref_link reference link to calculate pose
def get_jacobian(tip_link, urdf_content, Q, joint_names, ref_link="base_link"):

    Q_dict = list2dict(Q, joint_names)
    Jac = []
    chain = urdf_content.get_chain(root=ref_link, tip=tip_link)
    for ij, jname in enumerate(joint_names):
        if jname not in chain:
            Jac.append(np.zeros(6))
            continue
        joint = urdf_content.joint_map[jname]
        Tj = T_xyzrpy((joint.origin.xyz, joint.origin.rpy))
        T_link = get_tf(joint.parent, Q_dict, urdf_content, from_link=ref_link)
        T_bj = np.matmul(T_link, Tj)
        zi = np.matmul(T_bj[:3, :3], joint.axis)
        T_p = get_tf(tip_link, Q_dict, urdf_content, from_link=ref_link)
        dpi = T_p[:3, 3] - T_bj[:3, 3]
        zp = np.cross(zi, dpi)
        Ji = np.concatenate([zp, zi])
        Jac.append(Ji)
    Jac = np.array(Jac).transpose()
    return Jac