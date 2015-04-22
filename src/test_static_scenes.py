"""
Tests the validity of the static scenes library.
"""

from setup import *
from scene_generator import SceneGenerator
import PySCISim

import os
import numpy as np
import numpy.testing
from matplotlib import pyplot as plt
import unittest

#===============================================================================
# Control Variables
#===============================================================================
SCENE_FILE_PATH = "unittests"
H = 1.0/100
G = np.array([0.0, -9.81, 0.0])
SIM_STEPS = 1000
#===============================================================================
# UnitTests
#===============================================================================

class TestStaticSCISImScenes(unittest.TestCase):
    def test_free_fall_sphere(self):
        scenes_params = {
            "r": np.array([1]),
            "rho": np.array([1]),
            "x0": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "x1": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "h": np.array([H]),
            "g": G,
        }

        sim = PySCISim.SCISim()
        x = scenes_params["x0"].flatten()
        v = sim.get_dxdt_from_x(scenes_params["x0"],
                                scenes_params["x1"],
                                H
                                ).flatten()

        # generate an XML scene
        scene = SceneGenerator()

        scene.add_integrator(dt=str(H))
        scene.add_impact_operator()
        scene.add_friction_operator()
        scene.add_near_earth_gravity(f=G)
        scene.add_sphere(name="ball", r=scenes_params["r"][0])
        scene.add_rigid_body_with_density(geometry_name="ball",
                                            x=x[0:3],
                                            R=x[3:6],
                                            v=v[0:3],
                                            omega=v[3:6],
                                            rho=scenes_params["rho"][0],
                                            fixed="0")

        xml_scene_fname = os.path.join(SCENE_FILE_PATH, "free_fall_sphere.xml")
        scene.save(fname=xml_scene_fname)

        # load XML scene
        xml_sim = PySCISim.SCISim()
        xml_sim.openScene(xml_scene_fname)

        # load static scene
        # import pudb; pudb.set_trace()
        sim.loadScene("falling sphere", scenes_params, True)

        for i in range(SIM_STEPS):
            xml_sim.stepSystem()
            sim.stepSystem()
            xml_x = xml_sim.get_x().flatten()
            sim_x = sim.get_x().flatten()
            np.testing.assert_array_equal(xml_x, sim_x,
                 "Mismatch in values in step %i. (XML vs static)" % (i,))


if __name__ == '__main__':
    unittest.main()