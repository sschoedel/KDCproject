from meshcat_utils import CoordFrame

import meshcat
import meshcat.geometry as geom
from meshcat.animation import Animation

import os
import numpy as np
import time
        
        
class ArmVisualizer():
    """
    Visualizer abstraction so arm.py doesn't have unnecessary
    external dependencies
    Uses meshcat for visualization
    """
    
    robot_group = "robot/"
    
    def __init__(self, arm):
        self.arm = arm
    
    def startMeshcat(self):
        self.vis = meshcat.Visualizer()
        time.sleep(.5) # Sleeping gives better odds the visualizer is open before loading objects
        self.vis.open()
        time.sleep(.5) # Sleeping gives better odds the visualizer is open before loading objects
        self.vis.wait() # vis.wait() doesn't always work
        # time.sleep(.5) # Sleeping gives better odds the visualizer is open before loading objects
        self.setScene()
        time.sleep(.5) # Sleeping gives better odds the visualizer is open before loading objects
        self.loadArm()
        
    def setScene(self):
        self.vis["/Background"].set_property("top_color", (0.0, 0.0, 0.0))
    
    def loadArm(self, verbosity=0):
        # Load arm state from Arm() object
        # Visualize arm model in default position
        # Get useful member variables from Arm
        robot_urdf = self.arm.robot_urdf
        # Convert robot_urdf.materials list to dictionary
        material_color_dict = {}
        for material in robot_urdf.materials:
            material_color_dict[material.name] = material
            
        for i, link_name in enumerate(robot_urdf.link_map):
            # Get filepath for link mesh
            mesh_filepath = ""
            material_color = 0xaaaaaa
            if robot_urdf.link_map[link_name].collisions:
                # Filepath
                mesh_filepath = robot_urdf.link_map[link_name].collisions[0].geometry.filename
            #  Get material color for link mesh
            if robot_urdf.link_map[link_name].visuals:
                # Material color
                material_name = robot_urdf.link_map[link_name].visuals[0].material.name
                if material_name in material_color_dict:
                    material = material_color_dict[material_name]
                    r, g, b, a = material.color.rgba
                    # r,g,b in range [0,1] but material_color needs to be hex
                    material_color = (int(r*255)<<16) + (int(g*255)<<8) + int(b*255)
            
            # Set MeshCat link object path
            link_path = "robot/" + link_name
            # Create link mesh from filepath if exists
            if mesh_filepath != "":
                name, ext = os.path.splitext(mesh_filepath)
                if ext == ".stl":
                    self.vis[link_path].set_object(
                        geom.StlMeshGeometry.from_file(mesh_filepath),
                        geom.MeshLambertMaterial(color=material_color, reflectivity=0.3)
                    )
                elif ext == ".dae":
                    self.vis[link_path].set_object(
                        geom.DaeMeshGeometry.from_file(mesh_filepath),
                        geom.MeshLambertMaterial(color=material_color, reflectivity=0.3)
                    )
                elif ext == ".obj":
                    self.vis[link_path].set_object(
                        geom.ObjMeshGeometry.from_file(mesh_filepath),
                        geom.MeshLambertMaterial(color=material_color, reflectivity=0.3)
                    )
                
                # Initialize joint position to location specified by current arm configuration
                if link_name in self.arm.link_to_joint_name:
                    g = self.arm.gs_home[self.arm.link_to_joint_name[link_name]]
                    time.sleep(.2)
                    # print(f"setting {link_name}'s transformation: \n{g}")
                elif i == 0:
                    g = self.arm.gs_home[link_name]
                else:
                    print(f"{link_name} not in self.arm.link_to_joint_name map")
                    g = np.eye(4)
                self.vis[link_path].set_transform(g)
                if verbosity > 0: print(f"Set mesh for {link_name}")
            else:
                if verbosity > 0: print(f"No mesh filepath for {link_name}")
            # if i == 9:
                # break
        
    def updateLinkPoses(self, vis=None):
        """
        Changes pose of visual mesh for each link in MeshCat viewer

        Args:
            vis (meshcat.Visualizer(), optional): Allows for passing in an animation frame. Defaults to self.vis.
        """
        if not vis:
            vis = self.vis
        # Set joint position to location specified by current arm configuration
        for link_name in self.arm.robot_urdf.link_map:
            # Check if this link has a mesh to transform
            if self.arm.robot_urdf.link_map[link_name].collisions:
                # print(self.arm.link_to_joint_name)
                if link_name in self.arm.link_to_joint_name:
                    # Get transformation of joint associated with current link
                    g = self.arm.gs[self.arm.link_to_joint_name[link_name]]
                    vis[self.robot_group + link_name].set_transform(g)
    
    def animateArm(self, trajectory, timescale=1):
        """
            trajectory (np.array((N x m))): Rotation (theta) or displacement (d) for each joint. 
                                            N is the number of frames and m is the number of joints.
            timescale (int, optional): Divides the number of animation frames by this number
        """
        if timescale < 1:
            print("Error in animateArm: timescale < 1")
            return 0
        
        # self.vis["sphere"].set_object(geom.Sphere(.02), 
        #                                 geom.MeshLambertMaterial(
        #                                     color=0xff22dd,
        #                                     reflectivity=0.8))
        
        coordFrame = CoordFrame(self.vis, "frame1", length=0.05, radius=0.005)
        
        anim = Animation()
        for i, traj in enumerate(trajectory[::timescale]):
            with anim.at_frame(self.vis, i) as frame:
                # Generate trajectory
                self.arm.computeJointTransforms(traj, 0)
                tip_transform = self.arm.getArmTipTransform(traj)
                coordFrame.set_transform(tip_transform, vis=frame)
                # Apply transforms to links
                self.updateLinkPoses(frame)
                
        self.vis.set_animation(anim)