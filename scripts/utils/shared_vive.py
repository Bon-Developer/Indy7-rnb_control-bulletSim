import os
import sys
import subprocess
RNB_TRACK_DIR = os.path.join(os.environ['RNB_TRACK_DIR'])
sys.path.append(RNB_TRACK_DIR)

from shared_function import shared_fun, CallType, ArgSpec, ResSpec, serve_forever, SHARED_FUNC_ALL
from global_config import RNB_CONTROL_DIR
import numpy as np

if sys.version.startswith("3"):
    from rotation_utils import SE3_inv
    from triad_openvr import triad_openvr
else:
    from rotation_utils import SE3_inv

SERVER_ID = "SharedTrackers"


class SharedTrackers:
    def __init__(self):
        self.vive = None
        if __name__ != "__main__":
            output = subprocess.Popen(['python3',
                                       os.path.join(RNB_CONTROL_DIR, 'scripts/utils/shared_vive.py')])

    @shared_fun(CallType.SYNC, SERVER_ID)
    def init_vive(self):
        if self.vive is None:
            self.vive = triad_openvr.triad_openvr()
            self.vive.print_discovered_objects()
            self.tracker_names = self.vive.object_names["Tracker"] + self.vive.object_names["Controller"]
            self.ref_name = None
            self.tf_base_ref = None

    @shared_fun(CallType.SYNC, SERVER_ID,
                ArgSpec("ref_name", (32,), str),
                ArgSpec("tf_base_ref", (4,4), float))
    def set_reference(self, ref_name, tf_base_ref):
        if ref_name not in self.tracker_names:
            raise(RuntimeError("non-registered reference tracker name"))
        self.ref_name = ref_name
        self.tf_base_ref = np.copy(tf_base_ref)

    @shared_fun(CallType.SYNC, SERVER_ID,
                ResSpec(0, (32,), str),
                ResSpec(1, (4,4), float))
    def get_reference(self):
        return self.ref_name, self.tf_base_ref

    @shared_fun(CallType.SYNC, SERVER_ID,
                ResSpec(0, (10000,), dict))
    def get_all_pose(self):
        pose_dict = {}
        if self.ref_name is not None:
            if self.ref_name not in self.tracker_names:
                raise(RuntimeError("non-registered reference tracker name"))
            T_vref = self.__get_pose_4x4(self.ref_name)
            T_bv = np.matmul(self.tf_base_ref, SE3_inv(T_vref))
        else:
            T_bv = np.identity(4)

        for tname in self.tracker_names:
            pose = self.__get_pose_4x4(tname)
            pose_dict[tname] = np.matmul(T_bv, pose)
        return pose_dict

    def __get_pose_4x4(self, tname):
        pose = self.vive.devices[tname].get_pose_matrix()
        T = np.identity(4)
        T[:3,:] = np.asarray(pose.m)
        return T

    @shared_fun(CallType.SYNC, SERVER_ID,
                ArgSpec("tname", (32,), str),
                ResSpec(0, (4,4), float))
    def get_pose(self, tname):
        if self.ref_name is not None:
            if self.ref_name not in self.tracker_names:
                raise(RuntimeError("non-registered reference tracker name"))
            T_vref = self.__get_pose_4x4(self.ref_name)
            T_bv = np.matmul(self.tf_base_ref, SE3_inv(T_vref))
        else:
            T_bv = np.identity(4)
        if tname not in self.vive.devices:
            print("{} not in devices".format(tname))
            return None
        else:
            return np.matmul(T_bv, self.__get_pose_4x4(tname))


if __name__ == "__main__":
    vvt = SharedTrackers()
    serve_forever(SERVER_ID,
                  [vvt.init_vive,
                   vvt.set_reference,
                   vvt.get_reference,
                   vvt.get_all_pose,
                   vvt.get_pose], verbose=True)
