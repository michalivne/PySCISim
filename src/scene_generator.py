"""
Allows generation of XML scene files to  be used as SCISim input.
"""
import lxml.etree as et
from lxml.etree import Element
from lxml.etree import ElementTree
from lxml.etree import SubElement

import numpy as np

class SceneGenerator(object):
    """
    Generates XML scene files for SCISim.
    """
    def __init__(self):
        self.root = Element("threed_rigid_body_scene")
        self.tree = ElementTree(element=self.root)
        
        # holds a mapping between a name, an index and a geometry object
        self.geometry_ind_map = {}
        self.geometry_list = []

        self.static_plane_counter = 0

    def get_string(self, x):
        """
        Converts different objects to strings.
        """
        if type(x) == np.ndarray:
            s = str(x)[1:-1]
        elif type(x) == list:
            s = str(x)[1:-1].replace(",", "")
        else:
            s =  str(x)
            
        return s

    def add_componenet(self, tag, parent=None, **attributes):
        """
        A generic component insertion. 
        """
        if parent == None:
            parent = self.root
        # make sure all elements are string
        return SubElement(parent, tag,
                     attrib={k: self.get_string(v) 
                             for k, v in attributes.iteritems()})

    def add_integrator(self, type="split_ham", dt="0.01"):
        return self.add_componenet(tag="integrator",
                            type=type, dt=dt)

    def add_solver(self, parent_name, name="ipopt",
                   linear_solvers="ma97 ma57 mumps ma27 ma86",
                   con_tol="1.0e-9"):
        parent = self.root.find(parent_name)
        if parent == None:
            raise ValueError("Could not find element: %s" % parent_name)
        
        return self.add_componenet(tag="solver", parent=parent,
                            name=name, linear_solvers=linear_solvers,
                            con_tol=con_tol)

    def add_impact_operator(self, type="lcp", CoR="0.5", solver_params={}):
        e = self.add_componenet(tag="impact_operator", type=type, CoR=CoR)
        
        self.add_solver(parent_name="impact_operator", **solver_params)
        
        return e

    def add_friction_operator(self,
                              mu="0.8", type="linearized", disk_samples="1",
                              rel_sp_tol="1.0e-4", abs_sp_tol="1.0e-4",
                              max_iters="50", staggering="geometric",
                              solver_params={}):
        e = self.add_componenet(tag="friction_operator", mu=mu, type=type, 
                                disk_samples=disk_samples,
                                rel_sp_tol=rel_sp_tol, abs_sp_tol=abs_sp_tol,
                                max_iters=max_iters, staggering=staggering)
        
        self.add_solver(parent_name="friction_operator", **solver_params)
        
        return e
    
    def add_near_earth_gravity(self, f="0.0 -9.81 0.0"):
        return self.add_componenet(tag="near_earth_gravity",
                            f=f)
        
    def add_geometry(self, name, type, geometry_params={}):
        e = self.add_componenet(tag="geometry",
                            type=type, **geometry_params)
        self.geometry_ind_map[name] = len(self.geometry_list)
        self.geometry_list.append(e)
        
        return e
    
    def add_sphere(self, name, r="0.2"):
        self.add_geometry(name=name, type="sphere", 
                          geometry_params={"r": r})

    def add_box(self, name, r="0.1 0.2 0.3"):
        self.add_geometry(name=name, type="box", 
                          geometry_params={"r": r})

    def add_mesh(self, name, filename):
        self.add_geometry(name=name, type="mesh", 
                          geometry_params={"filename": filename})
    
    def add_static_plane(self, x="0.0 0.0 0.0", n="0.0 1.0 0.0", r="10.0 5.0"):
        e = self.add_componenet(tag="static_plane", x=x, n=n)
        self.add_componenet(tag="static_plane_renderer", 
                            plane=self.static_plane_counter, r=r)
        self.static_plane_counter += 1
        
        return e
    
    def add_rigid_body_with_density(self, geometry_name, x="0.0 0.0 0.0",
                            R="0 0 0", 
                            v="0.0 0.0 0.0",
                            omega="0.0 0.0 0.0",
                            rho="1.0",
                            fixed="0"):
        self.add_componenet(tag="rigid_body_with_density", 
                            x=x, R=R, v=v, omega=omega, rho=rho, fixed=fixed, 
                            geo_idx=self.geometry_ind_map[geometry_name])
    
    def save(self, fname, verbose=True):
        """
        Saves current scene.
        """
        if verbose:
            print "Writing SCISim XML file: %s" % (fname)

        self.tree.write(fname, pretty_print=True)
