import meshcat.geometry as geom
import meshcat.transformations as tf

import numpy as np

class Arrow():
    def __init__(self, vis, name, length=0.1, radius=0.01, color=0x0000ff, reflectivity=0.8):
        self.vis = vis
        self.name = name
        self.length = length
        self.color = color
        self.reflectivity = reflectivity
        self.cylinder = geom.Cylinder(length, radius=radius)
        self.cone = geom.Cylinder(radius*2, radiusBottom=radius*2, radiusTop=0)
        
        self.vis[self.name]["cylinder"].set_object(self.cylinder, 
                                                    geom.MeshLambertMaterial(color=color, reflectivity=reflectivity))
        self.vis[self.name]["cone"].set_object(self.cone, 
                                                geom.MeshLambertMaterial(color=color, reflectivity=reflectivity))
        
        self.g_cylinder = tf.rotation_matrix(-np.pi/2, np.array([0, 0, 1]), 
                                             np.array([0, 0, 0])) @ tf.translation_matrix([0, self.length/2, 0])
        self.g_cone = tf.rotation_matrix(-np.pi/2, np.array([0, 0, 1]), 
                                         np.array([0, 0, 0])) @ tf.translation_matrix([0, self.length, 0])
        
        self.vis[self.name]["cylinder"].set_transform(self.g_cylinder)
        self.vis[self.name]["cone"].set_transform(self.g_cone)
        
    def set_transform(self, g, vis=None):
        if vis == None:
            vis = self.vis
        vis[self.name]["cylinder"].set_transform(g @ self.g_cylinder)
        vis[self.name]["cone"].set_transform(g @ self.g_cone)
        
        
class CoordFrame():
    def __init__(self, vis, name, length=0.1, radius=0.01, reflectivity=0.8):
        self.vis = vis
        self.name = name
        self.length = length
        self.reflectivity = reflectivity
        self.arrow_x = Arrow(vis, name + "_arrow_x", length=length, radius=radius, color=0xff0000, reflectivity=reflectivity)
        self.arrow_y = Arrow(vis, name + "_arrow_y", length=length, radius=radius, color=0x00ff00, reflectivity=reflectivity)
        self.arrow_z = Arrow(vis, name + "_arrow_z", length=length, radius=radius, color=0x0000ff, reflectivity=reflectivity)

    def set_transform(self, g, vis=None):
        if vis == None:
            vis = self.vis
        self.arrow_x.set_transform(g, vis=vis)
        self.arrow_y.set_transform(g @ tf.rotation_matrix(np.pi/2, np.array([0, 0, 1]), np.array([0, 0, 0])), vis=vis)
        self.arrow_z.set_transform(g @ tf.rotation_matrix(-np.pi/2, np.array([0, 1, 0]), np.array([0, 0, 0])), vis=vis)

