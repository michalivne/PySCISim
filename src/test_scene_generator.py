"""
A test script for generating XML files for SCISim.
"""

from setup import *
from scene_generator import SceneGenerator

import numpy as np
from contexttimer import Timer

#===============================================================================
# Control variables
#===============================================================================
XML_FILE_NAME = "scene.xml"

#===============================================================================
# Main script
#===============================================================================
scene = SceneGenerator()

scene.add_integrator(dt="0.01")
scene.add_impact_operator()
scene.add_friction_operator()
scene.add_near_earth_gravity()
scene.add_sphere(name="ball", r=0.2)
scene.add_static_plane(x="0.0 0.0 0.0", n="1.0 1.0 0.0", r="10.0 5.0")
x = np.array([0.0, 2.0, 0.0]) 
v = [0.0, 1.0, 0.0]

for i in range(-1, 2):
    scene.add_rigid_body_with_density(geometry_name="ball",
    #                                     x="0.0 2.0 0.0",
                                        x=x+np.array([i, 0, 0]),
    #                                     v="0.0 0.0 0.0",
                                        v=v+np.array([-float(i)*2, 0, 0]),
                                        omega="0.0 0.0 0.0",
                                        rho="1.74040",
                                        fixed="0")

with Timer() as dt:
    scene.save(XML_FILE_NAME)

print "Saved a scene in %.2e [Sec]" % dt.elapsed

app = PySCISim.SCISimApp(False)
# app.openScene("scene.xml")
app.run("scene.xml")