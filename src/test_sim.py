"""
Tests PySCISim.
"""

from setup import *

import numpy as np
from matplotlib import pyplot as plt

#===============================================================================
# Control variables
#===============================================================================
SCENE_NAME = "GravityTest"
# SCENE_NAME = "SphereStaticPlaneTest"

SCENE_FILE_NAME = os.path.join(SCISIM_3D_ASSSETS_PATH, "Balls",  SCENE_NAME+".xml")
#===============================================================================
# Main script
#===============================================================================
app = PySCISim.SCISimApp(False)
app.openScene(SCENE_FILE_NAME)

sim = PySCISim.SCISim()
sim.openScene(SCENE_FILE_NAME)

# app.run()

# runs a short simulation
all_time = []
all_T = []
all_U = []
all_p = []
all_L = []

all_q = []
all_v = []

for i in range(100):
    app.stepSystem()
    sim.stepSystem()

    all_time.append(app.getSim_time())
    all_T.append(app.getSim_T())
    all_U.append(app.getSim_U())
    all_p.append(app.getSim_p().flatten())
    all_L.append(app.getSim_L().flatten())
    all_q.append(app.getSimState_q().flatten())
    all_v.append(app.getSimState_v().flatten())

    # test sim and app
    for method_name in ["getSim_time", "getSim_T", "getSim_U", "getSim_p",
                        "getSim_L", "getSimState_q", "getSimState_v"]:
        sim_v = getattr(sim, method_name)()
        app_v = getattr(app, method_name)()
        if not np.all(sim_v == app_v):
            raise ValueError("sim and app are not equal for method '%s'. sim: %s     app: %s" %\
                             (method_name, str(sim_v), str(app_v)))
    
# make all arrays numpy
all_time = np.array(all_time) 
all_T = np.array(all_T)
all_U = np.array(all_U)
all_p = np.array(all_p)
all_L = np.array(all_L)

all_q = np.array(all_q)
all_v = np.array(all_v)

# plot results

# energy and momentum
plt.figure()
plt.suptitle(SCENE_NAME)
plt.subplot(3, 1, 1)
plt.hold(True)
plt.plot(all_time, all_T, 'r', label="T")
plt.plot(all_time, all_U, 'g', label="U")
plt.grid(True)
plt.legend()
plt.ylabel("Energy")

plt.subplot(3, 1, 2)
plt.hold(True)
plt.plot(all_time, all_p)
plt.grid(True)
plt.ylabel("Momentum")

plt.subplot(3, 1, 3)
plt.hold(True)
plt.plot(all_time, all_L)
plt.grid(True)
plt.ylabel("Angular Momentum")

plt.xlabel("Time [Sec]")

# position and speed
plt.figure()
plt.suptitle(SCENE_NAME)
plt.subplot(2, 1, 1)
plt.hold(True)
plt.plot(all_time, all_q)
plt.grid(True)
plt.ylabel("q")

plt.subplot(2, 1, 2)
plt.hold(True)
plt.plot(all_time, all_v)
plt.grid(True)
plt.ylabel("v")

plt.xlabel("Time [Sec]")

plt.figure()
plt.imshow(app.getSimState_M())
plt.title(SCENE_NAME+" : M")
plt.show()
