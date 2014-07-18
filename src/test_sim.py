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

#===============================================================================
# Main script
#===============================================================================
app = PySCISim.SCISimApp()
app.openScene(os.path.join(SCISIM_3D_ASSSETS_PATH, "Balls", 
                           SCENE_NAME+".xml"))

# app.run()

# runs a short simulation
all_time = []
all_T = []
all_U = []
all_p = []
all_L = []

for i in range(100):
    app.stepSystem()
    all_time.append(app.getSim_time())
    all_T.append(app.getSim_T())
    all_U.append(app.getSim_U())
    all_p.append(app.getSim_p().flatten())
    all_L.append(app.getSim_L().flatten())
    
# make all arrays numpy
all_time = np.array(all_time) 
all_T = np.array(all_T)
all_U = np.array(all_U)
all_p = np.array(all_p)
all_L = np.array(all_L)

# plot results
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

plt.show()