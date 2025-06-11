import os
import sys

ROOT_NAME = "rnb-control"
if ROOT_NAME in __file__:
    RNB_CONTROL_DIR = __file__[:__file__.find(ROOT_NAME)+len(ROOT_NAME)]
elif ROOT_NAME in os.getcwd():
    CWD_DIR = os.getcwd()
    RNB_CONTROL_DIR = CWD_DIR[:CWD_DIR.find(ROOT_NAME)+len(ROOT_NAME)]
else:
    raise(RuntimeError("Unsolvable directory"))