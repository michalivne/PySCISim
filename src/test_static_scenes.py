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
import time

#===============================================================================
# Control Variables
#===============================================================================
PLOT_SIM = True

SCENE_FILE_PATH = "unittests"
H = 1.0/100
G = np.array([0.0, -9.81, 0.0])
SIM_STEPS = 500

#===============================================================================
# Functions
#===============================================================================
def compare_sim(xml_sim, sim, step_num, scene_name):
    """
    Compares two input simulations.
    """
    t0 = time.time()
    all_x = []
    for i in range(step_num):
        xml_sim.stepSystem()
        sim.stepSystem()
        xml_x = xml_sim.get_x().flatten()
        sim_x = sim.get_x().flatten()

        all_x.append(sim_x)
        # should be almost equal since rotation is translated to/from matrices
        np.testing.assert_array_almost_equal(xml_x, sim_x, decimal=5,
             err_msg="Mismatch in values in step %i. (XML vs static)" % (i,))

    dt = time.time() - t0
    fps = float(step_num) / dt
    print ">>>>>>> Done %s in %.4f [Sec] (%.2f FPS)" % (scene_name, dt, fps)

    if PLOT_SIM:
        plt.figure()
        plt.grid(True)
        plt.title(scene_name)
        plt.xlabel('Time Step #')
        plt.ylabel('Sim x')
        plt.plot(all_x)

def get_x_v(scene_params):
    """
    Return XML friendly x, v from scene_params.
    """
    sim = PySCISim.SCISim()
    x = scene_params["x0"].flatten()
    v = sim.get_dxdt_from_x(scene_params["x0"],
                    scene_params["x1"],
                    H
                    ).flatten()


    # truncate accuracy to support XML
    x = np.fromstring(str(x).strip('[').strip(']'), sep=' ')
    v = np.fromstring(str(v).strip('[').strip(']'), sep=' ')

    return x, v

#===============================================================================
# UnitTests
#===============================================================================

class TestStaticSCISImScenes(unittest.TestCase):
    def test_free_fall_sphere(self):
        scene_params = {
            "r": np.array([1.0]),
            "rho": np.array([1.0]),
            "x0": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "x1": np.array([0.01, 0.02, 0.03, 0.04, 0.05, 0.06]),
            "h": np.array([H]),
            "g": G,
        }

        x, v = get_x_v(scene_params)

        # generate an XML scene
        scene = SceneGenerator()

        scene.add_integrator(dt=str(H))
        scene.add_impact_operator()
        scene.add_friction_operator()
        scene.add_near_earth_gravity(f=G)
        scene.add_sphere(name="ball", r=scene_params["r"][0])
        scene.add_rigid_body_with_density(geometry_name="ball",
                                            x=x[0:3],
                                            R=x[3:6],
                                            v=v[0:3],
                                            omega=v[3:6],
                                            rho=scene_params["rho"][0],
                                            fixed="0")

        scene_name = "falling sphere"
        xml_scene_fname = os.path.join(SCENE_FILE_PATH,
                                       "_".join(scene_name.split(" "))+".xml")
        scene.save(fname=xml_scene_fname)

        # load XML scene
        xml_sim = PySCISim.SCISim()
        xml_sim.openScene(xml_scene_fname)

        # load static scene
        sim = PySCISim.SCISim()
        sim.loadScene(scene_name, scene_params, True)

        compare_sim(xml_sim, sim, SIM_STEPS, scene_name)

    def test_sphere_on_plane(self):
        scene_params = {
            "r": np.array([1.0]),
            "rho": np.array([1.0]),
            "p0": np.array([0.0, -2.0, 0.0]),
            "n": np.array([0.3, 1.0, 0.0]),
            "x0": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "x1": np.array([0.01, 0.02, 0.03, 0.04, 0.05, 0.06]),
            "CoR": np.array([0.5]),
            "mu": np.array([0.3]),
            "h": np.array([H]),
            "g": G,
        }

        scene_params["n"] /= np.sqrt(np.sum(scene_params["n"]**2))

        x, v = get_x_v(scene_params)

        # generate an XML scene
        scene = SceneGenerator()

        scene.add_integrator(dt=str(H))
        scene.add_impact_operator(CoR=scene_params["CoR"][0])
        scene.add_friction_operator(mu=scene_params["mu"][0])
        scene.add_near_earth_gravity(f=G)
        scene.add_sphere(name="ball", r=scene_params["r"][0])
        scene.add_rigid_body_with_density(geometry_name="ball",
                                            x=x[0:3],
                                            R=x[3:6],
                                            v=v[0:3],
                                            omega=v[3:6],
                                            rho=scene_params["rho"][0],
                                            fixed="0")
        scene.add_static_plane(x=scene_params["p0"],
                               n=scene_params["n"],
                               r="10.0 5.0")

        scene_name = "sphere on plane"
        xml_scene_fname = os.path.join(SCENE_FILE_PATH,
                                       "_".join(scene_name.split(" "))+".xml")
        scene.save(fname=xml_scene_fname)

        # load XML scene
        xml_sim = PySCISim.SCISim()
        xml_sim.openScene(xml_scene_fname)

        # load static scene
        sim = PySCISim.SCISim()
        sim.loadScene(scene_name, scene_params, True)

        compare_sim(xml_sim, sim, SIM_STEPS, scene_name)

    def test_box_on_plane(self):
        scene_params = {
            "r": np.array([0.3, 0.5, 1.0]),
            "rho": np.array([1.0]),
            "p0": np.array([0.0, -2.0, 0.0]),
            "n": np.array([0.3, 1.0, 0.0]),
            "x0": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "x1": np.array([0.01, 0.02, 0.03, 0.04, 0.05, 0.06]),
            "CoR": np.array([0.5]),
            "mu": np.array([0.3]),
            "h": np.array([H]),
            "g": G,
        }

        scene_params["n"] /= np.sqrt(np.sum(scene_params["n"]**2))

        x, v = get_x_v(scene_params)

        # generate an XML scene
        scene = SceneGenerator()

        scene.add_integrator(dt=str(H))
        scene.add_impact_operator(CoR=scene_params["CoR"][0])
        scene.add_friction_operator(mu=scene_params["mu"][0])
        scene.add_near_earth_gravity(f=G)
        scene.add_box(name="box", r=scene_params["r"])
        scene.add_rigid_body_with_density(geometry_name="box",
                                            x=x[0:3],
                                            R=x[3:6],
                                            v=v[0:3],
                                            omega=v[3:6],
                                            rho=scene_params["rho"][0],
                                            fixed="0")
        scene.add_static_plane(x=scene_params["p0"],
                               n=scene_params["n"],
                               r="10.0 5.0")

        scene_name = "box on plane"
        xml_scene_fname = os.path.join(SCENE_FILE_PATH,
                                       "_".join(scene_name.split(" "))+".xml")
        scene.save(fname=xml_scene_fname)

        # load XML scene
        xml_sim = PySCISim.SCISim()
        xml_sim.openScene(xml_scene_fname)

        # load static scene
        sim = PySCISim.SCISim()
        sim.loadScene(scene_name, scene_params, True)

        compare_sim(xml_sim, sim, SIM_STEPS, scene_name)


if __name__ == '__main__':
    unittest.main()