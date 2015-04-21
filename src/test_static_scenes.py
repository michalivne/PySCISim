"""
Tests the validity of the static scenes library.
"""

from setup import *
from scene_generator import SceneGenerator

import numpy as np
from matplotlib import pyplot as plt

import unittest


class TestStaticSCISImScenes(unittest.TestCase):
    def test_free_fall_sphere(self):
        scene = SceneGenerator()

        scene.add_integrator(dt="0.01")
        scene.add_impact_operator()
        scene.add_friction_operator()
        scene.add_near_earth_gravity()
        scene.add_sphere(name="ball", r=0.2)
        scene.add_static_plane(x="0.0 0.0 0.0", n="1.0 1.0 0.0", r="10.0 5.0")
        x = np.array([0.0, 2.0, 0.0])
        v = [0.0, 1.0, 0.0]


if __name__ == '__main__':
    unittest.main()