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

MACHINE_NAME = socket.gethostname()
