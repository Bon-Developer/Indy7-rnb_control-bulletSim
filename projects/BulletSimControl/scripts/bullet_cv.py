from bullet_sim_client import *
import cv2
from cv2 import aruco
import pybullet as pb

ARUCO_DICTIONARY = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

def get_marker(mk_idx):
    return aruco.drawMarker(ARUCO_DICTIONARY, mk_idx, ARUCO_DICTIONARY.markerSize+2)

##
# @brief add marker in bullet environment
# @param mid marker id
# @param Tbm transformation of marker in world coords. 
#            local axis: image-up=x, image-left=y
# @returns object id lists for marker, to remove with pb.remove_body()
def add_marker(mid, Tbm, marker_dim, thickness = 0.01):
    img = get_marker(mid).astype(bool)
    im_size=img.shape[0] # before pad
    cell_dim = np.divide(marker_dim, im_size)
    
    img = np.pad(img, (1,1), 'constant', constant_values=True)
    marker_dim += cell_dim*2
    im_size=img.shape[0] # after pad
    
    gid_list = []
    for i in range(im_size):
        for j in range(im_size):
            pt_off = (marker_dim/im_size*(i+0.5), marker_dim/im_size*(j+0.5))
            Toff = SE3(np.identity(3), 
                       np.subtract((marker_dim/2, marker_dim/2, -thickness/2),
                                  (pt_off[0], pt_off[1], 0)))
            Tbij = np.matmul(Tbm, Toff)
            xyz, quat = T2xyzquat(Tbij)
            gid = add_simple_geom(pb.GEOM_BOX, xyz, quat, 
                                  mass=0, halfExtents=[cell_dim/2, cell_dim/2, thickness/2],
                                  rgbaColor=(img[i,j],)*3+(1,), collision=False)
            gid_list.append(gid)
    return gid_list

##
# @brief calculate camera transformation matrix
# @param cam_pos camera position in world coordinate
# @param cam_tar viewpoint target in world coordinate
# @param up_vec  reference direction of image up-side (y-axis) in word coords
def calc_Tbc(cam_pos, cam_tar, up_vec=[0,0,1]):
    view_matrix = pb.computeViewMatrix(
        cameraEyePosition=cam_pos,
        cameraTargetPosition=cam_tar,
        cameraUpVector=up_vec)
    return np.linalg.inv(np.reshape(view_matrix, (4,4)).transpose())

##
# @param Tbc camera transformation matrix in world coordinates. +z is camera back, +y is up direction in the image
# @returns rgba color image and depthmap
def get_cam_images(Tbc,
                   im_wh=(1920,1080),fov=60, nearVal=0.1, farVal=100.0):
    view_matrix = np.linalg.inv(Tbc).transpose().flatten().tolist()
    proj_matrix = pb.computeProjectionMatrixFOV(
        fov=fov, aspect=float(im_wh[0])/im_wh[1], 
        nearVal=nearVal, farVal=farVal)

    (_, _, color, depth, _) = pb.getCameraImage(
        width=im_wh[0], height=im_wh[1], viewMatrix=view_matrix,
        projectionMatrix=proj_matrix)
    color = np.reshape(color, tuple(reversed(im_wh))+(4,))
    depth = np.reshape(depth, tuple(reversed(im_wh)))
    
    proj_matrix = np.reshape(proj_matrix, (4,4))
    fx = proj_matrix[0,0]*im_wh[0]/2
    fy = proj_matrix[1,1]*im_wh[1]/2
    cameraMatrix = np.array([[fx, 0, im_wh[0]/2],
                             [0, fy, im_wh[1]/2],
                             [0,0,1]])
    return color, depth, cameraMatrix