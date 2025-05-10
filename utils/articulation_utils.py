import sapien
import numpy as np
from copy import deepcopy
import transforms3d as t3d
from ..robot import Robot
from pathlib import Path
from typing import Literal
class ArticulationUtils:
    def __init__(self, 
                 actor : sapien.physx.PhysxArticulation,
                 robot : Robot,
                 config: dict):
        self.actor = actor
        self.robot = robot
        self.config = config
        
        self.link_dict = self.get_link_dict()
    
    def set_mass(self, mass):
        for link in self.actor.get_links():
            link.set_mass(mass)
    
    def set_properties(self, damping, stiffness, friction=None, force_limit=None):
        for joint in self.actor.get_joints():
            if force_limit is not None:
                joint.set_drive_properties(
                    damping=damping,
                    stiffness=stiffness,
                    force_limit=force_limit
                )
            else:
                joint.set_drive_properties(
                    damping=damping,
                    stiffness=stiffness,
                )
            if friction is not None:
                joint.set_friction(friction)

    def get_link_dict(self):
        link_dict = {}
        for link in self.actor.get_links():
            link_dict[link.get_name()] = link
        return link_dict
    
    def get_point(self, type:str, idx:int)\
        -> tuple[list, sapien.physx.PhysxArticulationLinkComponent]:
        type = {
            'contact': 'contact_points',
            'target' : 'target_points',
            'functional': 'functional_points',
            'orientation': 'orientation_point'
        }[type]
        matrix = np.array(self.config[type][idx]['matrix'])
        matrix[:3, 3] *= self.config['scale']
        return (
            matrix, 
            self.link_dict[self.config[type][idx]['base']]
        )
    
    @staticmethod
    def get_face_prod(q, local_axis, target_axis):
        '''
            get product of local_axis (under q world) and target_axis
        '''
        q_mat = t3d.quaternions.quat2mat(q)
        face = q_mat @ np.array(local_axis).reshape(3, 1)
        face_prod = np.dot(face.reshape(3), np.array(target_axis))
        return face_prod

    @staticmethod
    def get_debug_info(task):
        info = '[MODEL INFO]\n'
        info += f'NAME: {task.model_name}\n'
        if task.model_id is not None:
            modeldir = Path("assets") / "objects" / task.model_name
            model_list = [model for model in modeldir.iterdir() if model.is_dir()]
            info += f'  ID: ({task.model_id:02}){model_list[task.model_id].name}\n'
        info += '\n[TASK INFO]\n'
        if hasattr(task, 'lr_tag'):
            info += f'LR_TAG: {task.lr_tag}'
        return info
    
    @staticmethod
    def get_offset(pose, x=0, y=0, z=0):
        if type(pose) == sapien.Pose:
            pose = pose.p.tolist() + pose.q.tolist()
        pose[0] += x
        pose[1] += y
        pose[2] += z
        return pose
    
    @staticmethod
    def get_rotate(pose, x=0, y=0, z=0):
        if type(pose) == sapien.Pose:
            pose = pose.p.tolist() + pose.q.tolist()
        q = t3d.quaternions.quat2mat(pose[3:])
        q = t3d.quaternions.mat2quat(q @ t3d.euler.euler2mat(x, y, z))
        pose[3:] = q
        return pose
    
    # 2025.4.10
    # =========================================================== New APIS ===========================================================
    def get_target_pose_from_goal_point_and_gripper_direction(
        self, arm_tag = None, target_pose = None, target_grasp_qpose = None):
        """
            Obtain the grasp pose through the given target point and contact direction.
            - actor: The instance of the object to be grasped.
            - actor_data: The annotation data corresponding to the instance of the object to be grasped.
            - endpose: The end effector pose, from robot.get_left_ee_pose() or robot.get_right_ee_pose().
            - target_pose: The target point coordinates for aligning the functional points of the object to be grasped.
            - target_grasp_qpose: The direction of the grasped object's contact target point, 
                                 represented as a quaternion in the world coordinate system.
        """
        endpose = self.robot.get_left_ee_pose() if arm_tag == 'left' else self.robot.get_right_ee_pose()
        matrix, link = self.get_point('target', 0)
        actor_matrix = link.get_pose().to_transformation_matrix()
        local_target_matrix = np.array(matrix)

        res_matrix = np.eye(4)
        res_matrix[:3,3] = (actor_matrix  @ local_target_matrix)[:3,3] - endpose[:3]
        # @ np.array([[1,0,0],[0,-1,0],[0,0,-1]])
        res_matrix[:3,3] = np.linalg.inv(t3d.quaternions.quat2mat(endpose[-4:])) @ res_matrix[:3,3]
        res_pose = list(target_pose - t3d.quaternions.quat2mat(target_grasp_qpose) @ res_matrix[:3,3]) + target_grasp_qpose
        return res_pose

    def get_grasp_pose_w_labeled_direction(self, pre_dis = 0., contact_point_id = 0):
        """
            Obtain the grasp pose through the marked grasp point.
            - actor: The instance of the object to be grasped.
            - actor_data: The annotation data corresponding to the instance of the object to be grasped.
            - pre_dis: The distance in front of the grasp point.
            - contact_point_id: The index of the grasp point.
        """
        matrix, link = self.get_point('contact', contact_point_id)
        actor_matrix = link.get_pose().to_transformation_matrix()
        local_contact_matrix = np.array(matrix)
        global_contact_pose_matrix = actor_matrix  @ local_contact_matrix @ np.array([[0, 0, 1, 0],
                                                                                      [-1,0, 0, 0],
                                                                                      [0, -1,0, 0],
                                                                                      [0, 0, 0, 1]])
        global_contact_pose_matrix_q = global_contact_pose_matrix[:3,:3]
        global_grasp_pose_p = global_contact_pose_matrix[:3,3] + global_contact_pose_matrix_q @ np.array([-0.12-pre_dis,0,0]).T
        global_grasp_pose_q = t3d.quaternions.mat2quat(global_contact_pose_matrix_q)
        res_pose = list(global_grasp_pose_p)+list(global_grasp_pose_q)
        return res_pose
    
    def get_grasp_pose_from_goal_point_and_direction(
        self, endpose_tag: str, actor_functional_point_id = 0,
        target_point = None, target_approach_direction = [0,0,1,0],
        actor_target_orientation = None, pre_dis = 0.):
        """
            Obtain the grasp pose through the given target point and contact direction.
            - actor: The instance of the object to be grasped.
            - actor_data: The annotation data corresponding to the instance of the object to be grasped.
            - endpose_tag: Left and right gripper marks, with values "left" or "right".
            - actor_functional_point_id: The index of the functional point to which the object to be grasped needs to be aligned.
            - target_point: The target point coordinates for aligning the functional points of the object to be grasped.
            - target_approach_direction: The direction of the grasped object's contact target point, 
                                         represented as a quaternion in the world coordinate system.
            - actor_target_orientation: The final target orientation of the object, 
                                        represented as a direction vector in the world coordinate system.
            - pre_dis: The distance in front of the grasp point.
        """
        target_approach_direction_mat = t3d.quaternions.quat2mat(target_approach_direction)
        target_point_copy = deepcopy(target_point[:3])
        target_point_copy -= target_approach_direction_mat @ np.array([0,0,pre_dis])

        try:
            matrix, link = self.get_point('orientation', 0)
            actor_matrix = link.get_pose().to_transformation_matrix()
            actor_orientation_point = actor_matrix[:3, 3] + matrix[:3, 3]
        except:
            actor_matrix = self.actor.get_links()[0].get_pose().to_transformation_matrix()
            actor_orientation_point = [0,0,0]

        if actor_target_orientation is not None:
            actor_target_orientation = actor_target_orientation / np.linalg.norm(actor_target_orientation)

        end_effector_pose = self.robot.get_left_ee_pose() if endpose_tag == 'left' else self.robot.get_right_ee_pose()
        res_pose = None
        # res_eval= -1e10
        # for adjunction_matrix in adjunction_matrix_list:
        local_target_matrix, target_link = self.get_point('functional', actor_functional_point_id)
        target_link_matrix = target_link.get_pose().to_transformation_matrix()
        fuctional_matrix = target_link_matrix[:3, :3] @ local_target_matrix[:3,:3]
        # fuctional_matrix = fuctional_matrix @ adjunction_matrix
        trans_matrix = target_approach_direction_mat @ np.linalg.inv(fuctional_matrix)
        #  @ np.array([[1,0,0],[0,-1,0],[0,0,-1]])
        ee_pose_matrix = t3d.quaternions.quat2mat(end_effector_pose[-4:])
        target_grasp_matrix = trans_matrix @ ee_pose_matrix

        # Use actor target orientation to filter
        if actor_target_orientation is not None:
            now_actor_orientation_point = trans_matrix @ actor_matrix[:3,:3] @ np.array(actor_orientation_point)
            now_actor_orientation_point = now_actor_orientation_point / np.linalg.norm(now_actor_orientation_point)
            # produt = np.dot(now_actor_orientation_point, actor_target_orientation)
            # # The difference from the target orientation is too large
            # if produt < 0.8:
            #     continue
        
        res_matrix = np.eye(4)
        res_matrix[:3,3] = (target_link_matrix @ local_target_matrix)[:3,3] - end_effector_pose[:3]
        res_matrix[:3,3] = np.linalg.inv(ee_pose_matrix) @ res_matrix[:3,3]
        target_grasp_qpose = t3d.quaternions.mat2quat(target_grasp_matrix)
        # priget_grasp_pose_w_labeled_directionnt(target_grasp_matrix @ res_matrix[:3,3])
        res_pose = (target_point_copy - target_grasp_matrix @ res_matrix[:3,3]).tolist() + target_grasp_qpose.tolist()
        return res_pose
    
    # Get the pose coordinates of the actor's target point in the world coordinate system.
    # Return value: [x, y, z]
    def get_actor_target_pose(self, target_id = 0):
        matrix, link = self.get_point('target', target_id)
        actor_matrix = link.get_pose().to_transformation_matrix()
        return (actor_matrix @ matrix)[:3,3]

    # Get the actor's functional point and axis corresponding to the index in the world coordinate system.
    # Return value: [x, y, z, quaternion].
    def get_actor_functional_pose(self, functional_id = 0):
        matrix, link = self.get_point('functional', functional_id)
        actor_matrix = link.get_pose().to_transformation_matrix()
        # if "model_type" in actor_data.keys() and actor_data["model_type"] == "urdf": actor_matrix[:3,:3] = self.URDF_MATRIX
        res_matrix = actor_matrix @ matrix
        return res_matrix[:3,3].tolist() + t3d.quaternions.mat2quat(res_matrix[:3,:3]).tolist()

    # Get the actor's grasp point and axis corresponding to the index in the world coordinate system.
    # Return value: [x, y, z, quaternion]
    def get_actor_contact_pose(self, contact_id = 0):
        matrix, link = self.get_point('contact', contact_id)
        actor_matrix = link.get_pose().to_transformation_matrix()
        # if "model_type" in actor_data.keys() and actor_data["model_type"] == "urdf": actor_matrix[:3,:3] = self.URDF_MATRIX
        res_matrix = actor_matrix @ matrix
        return res_matrix[:3,3].tolist() + t3d.quaternions.mat2quat(res_matrix[:3,:3]).tolist()
    # =========================================================== New APIS ===========================================================