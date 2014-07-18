"""
Setup paths and common variables.
"""
import os
import sys
import glob
import socket

# add PySCISim Python bindings
sys.path.append(os.path.abspath("cpp/build/PySCISim"))
import PySCISim

SCISIM_PATH = os.path.abspath("../../SCISim")
SCISIM_ASSSETS_PATH = os.path.join(SCISIM_PATH, "assets")
SCISIM_3D_ASSSETS_PATH = os.path.join(SCISIM_ASSSETS_PATH, "3DRigidBodyScene")

MACHINE_NAME = socket.gethostname()
