import sapien.core as sapien
import numpy as np
import transforms3d as t3d
import sapien.physx as sapienp
from .create_actor import *

import re
import json
from pathlib import Path

def get_all_messy_objects():
    messy_objects_info = {}
    messy_objects_name = []
    
    # load from messy_objects
    messy_objects_config = json.load(
        open(Path("./assets/messy_objects/list.json"), 'r', encoding='utf-8'))
    messy_objects_name += messy_objects_config['item_names']
    for model_name, model_ids in messy_objects_config['list_of_items'].items():
        messy_objects_info[model_name] = {
            'ids' : model_ids,
            'type': 'urdf',
            'root': f'messy_objects/{model_name}'
        }
        params = {}
        for model_id in model_ids:
            model_full_name = f'{model_name}_{model_id}'
            params[model_id] = {
                'z_max'   : messy_objects_config['z_max'][model_full_name],
                'radius'  : messy_objects_config['radius'][model_full_name],
                'z_offset': messy_objects_config['z_offset'][model_full_name]
            }
        messy_objects_info[model_name]['params'] = params
    
    # load from objects
    objects_dir = Path("./assets/objects")
    for model_dir in objects_dir.iterdir():
        if not model_dir.is_dir(): continue
        if re.search(r'^(\d+)_(.*)', model_dir.name) is None: continue
        model_name = model_dir.name
        model_id_list, params = [], {}
        for model_cfg in model_dir.iterdir():
            if model_cfg.is_dir() or model_cfg.suffix != '.json': continue
            
            # get model id
            model_id = re.search(r'model_data(\d+)', model_cfg.name)
            if not model_id: continue
            model_id = model_id.group(1)
            
            # get model params
            model_config:dict = json.load(open(model_cfg, 'r', encoding='utf-8'))
            if 'center' not in model_config or 'extents' not in model_config: continue
            if model_config.get('stable', True) is False: continue
            center  = model_config['center']
            extents = model_config['extents']
            scale   = model_config.get('scale', [1., 1., 1.])
            # 0: x, 1: z, 2: y
            params[model_id] = {
                'z_max'   : (extents[1] + center[1]) * scale[1],
                'radius'  : max(extents[0]*scale[0], extents[2]*scale[2]) / 2,
                'z_offset': 0
            }
            model_id_list.append(model_id)
        if len(model_id_list) == 0: continue
        messy_objects_name.append(model_name)
        model_id_list.sort()
        messy_objects_info[model_name] = {
            'ids' : model_id_list,
            'type': 'glb',
            'root': f'objects/{model_name}',
            'params': params
        }
    
    same_obj = json.load(
        open(Path("./assets/messy_objects/same.json"), 'r', encoding='utf-8'))
    messy_objects_name = list(messy_objects_name)
    messy_objects_name.sort()
    return messy_objects_info, messy_objects_name, same_obj

messy_objects_info, messy_objects_list, same_obj = get_all_messy_objects()

def get_available_messy_objects(entity_on_scene:list):
    global messy_objects_info, messy_objects_list, same_obj
     
    model_in_use = []
    for entity_name in entity_on_scene:
        if same_obj.get(entity_name) is not None:
            model_in_use += same_obj[entity_name]
        model_in_use.append(entity_name)

    available_models = set(messy_objects_list) - set(model_in_use)
    available_models = list(available_models)
    available_models.sort()
    return available_models, messy_objects_info
    

def check_overlap(radius, x, y, area):
    if x <= area[0]:
        dx = area[0] - x
    elif area[0] < x and x < area[2]:
        dx = 0
    elif x >= area[2]:
        dx = x - area[2]
    if y <= area[1]:
        dy = area[1] - y
    elif area[1] < y and y < area[3]:
        dy = 0
    elif y >= area[3]:
        dy = y - area[3]
    
    return dx * dx + dy * dy <= radius * radius


def rand_pose_messy(
    xlim: np.ndarray,
    ylim: np.ndarray,
    zlim: np.ndarray,
    ylim_prop = False,
    rotate_rand = False,
    rotate_lim = [0,0,0],
    qpos = [1,0,0,0],
    size_dict = None,
    obj_radius = 0.1,
    z_offset = 0.001,
    z_max = 0,
    prohibited_area = None,
    obj_margin = 0.005
) -> sapien.Pose:  
    if (len(xlim)<2 or xlim[1]<xlim[0]):
        xlim=np.array([xlim[0],xlim[0]])
    if (len(ylim)<2 or ylim[1]<ylim[0]):
        ylim=np.array([ylim[0],ylim[0]])
    if (len(zlim)<2 or zlim[1]<zlim[0]):
        zlim=np.array([zlim[0],zlim[0]])
    
    times = 0
    while True:
        times += 1
        if times > 100:
            return False, None
        x = np.random.uniform(xlim[0], xlim[1])
        y = np.random.uniform(ylim[0], ylim[1])
        new_obj_radius = obj_radius + obj_margin
        is_overlap = False
        for area in prohibited_area:
            # if x + obj_radius >= area[0] and x - obj_radius <= area[1] and y - obj_radius >= area[2] and y + obj_radius <= area[3]:
            #     continue
            # if x > area[0] and x < area[1] and y > area[2] and y < area[3]:
            #     continue
            if check_overlap(new_obj_radius, x, y, area):
                is_overlap = True
                break
        if is_overlap:
            continue
        distances = np.sqrt((np.array([sub_list[0] for sub_list in size_dict]) - x) ** 2 + (np.array([sub_list[1] for sub_list in size_dict]) - y) ** 2)
        max_distances = np.array([sub_list[3] + new_obj_radius + obj_margin for sub_list in size_dict])

        if y - new_obj_radius < 0:
            if z_max > 0.05:
                continue
        if x - new_obj_radius < -0.6 or x + new_obj_radius > 0.6 or \
           y - new_obj_radius < -0.34 or y + new_obj_radius > 0.34:
            continue
        if np.all(distances > max_distances) and y + new_obj_radius < ylim[1]:
            break
        
    z = np.random.uniform(zlim[0],zlim[1])
    z = z - z_offset

    rotate = qpos
    if (rotate_rand):
        angles = [0,0,0]
        for i in range(3):
            angles[i] = np.random.uniform(-rotate_lim[i],rotate_lim[i])
        rotate_quat = t3d.euler.euler2quat(angles[0], angles[1], angles[2])
        rotate = t3d.quaternions.qmult(rotate, rotate_quat)

    return True,sapien.Pose([x, y, z],rotate)

def rand_create_messy_actor(
    scene: sapien.Scene,
    modelname: str,
    modelid  : str,
    modeltype: str,
    xlim: np.ndarray,
    ylim: np.ndarray,
    zlim: np.ndarray,
    ylim_prop = False,
    rotate_rand = False,
    rotate_lim = [0,0,0],
    qpos = None,
    scale = (1,1,1),
    convex = True,
    is_static = False,
    size_dict = None,
    obj_radius = 0.1,
    z_offset = 0.001,
    z_max = 0,
    fix_root_link = True,
    prohibited_area = None
) -> sapien.Entity:
    if qpos is None:
        if modeltype == 'glb':
            qpos = [0.707107, 0.707107, 0, 0]
            rotate_lim = [rotate_lim[0], rotate_lim[2], rotate_lim[1]]
        else:
            qpos = [1, 0, 0, 0]
 
    success, obj_pose = rand_pose_messy(
        xlim=xlim,
        ylim=ylim,
        zlim=zlim,
        ylim_prop=ylim_prop,
        rotate_rand=rotate_rand,
        rotate_lim=rotate_lim,
        qpos=qpos,
        size_dict=size_dict,
        obj_radius = obj_radius,
        z_offset=z_offset,
        z_max=z_max,
        prohibited_area=prohibited_area
    )
    if not success:
        return False, None
    
    if modeltype == 'urdf':
        obj, _ = create_messy_urdf_obj(
            scene,
            pose=obj_pose,
            modelname=f'messy_objects/{modelname}/{modelid}',
            scale=scale if isinstance(scale, float) else scale[0],
            fix_root_link=fix_root_link
        )
        if obj is None:
            return False, None
        else:
            obj.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
            return True, obj
    else:
        obj, _ = create_actor(
            scene=scene,
            pose=obj_pose,
            modelname=modelname,
            model_id=modelid,
            scale=scale,
            convex=convex,
            is_static=is_static,
        )
        if obj is None:
            return False, None
        else:
            obj.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
            return True, obj

def create_messy_urdf_obj(
    scene: sapien.Scene,
    pose: sapien.Pose,
    modelname: str,
    scale = 1.0,
    fix_root_link = True
)->tuple[sapienp.PhysxArticulation, dict]: 
    modeldir = "./assets/"+modelname+"/"
    file_name = modeldir + "base.glb"
    json_file_path = modeldir + 'model_data.json'
    
    try:
        with open(json_file_path, 'r') as file:
            model_data = json.load(file)
        scale = model_data["scale"]
    except:
        model_data = None

    loader: sapien.URDFLoader = scene.create_urdf_loader()
    loader.scale = scale
    loader.fix_root_link = fix_root_link
    loader.load_multiple_collisions_from_file = False
    modeldir = "./assets/"+modelname+"/"
    object: sapien.Articulation = loader.load_multiple(modeldir+"model.urdf")[1][0]
    object.set_pose(pose)
    return object, model_data


def rand_create_messy_urdf_obj(
    scene: sapien.Scene,
    modelname: str,
    xlim: np.ndarray,
    ylim: np.ndarray,
    zlim: np.ndarray,
    ylim_prop = False,
    rotate_rand = False,
    rotate_lim = [0,0,0],
    qpos = [1,0,0,0],
    scale = 1.0,
    fix_root_link = True,
    size_dict = None,
    obj_radius = 0.1,
    z_offset = 0.001,
    z_max = 0,
    prohibited_area = None
)->sapienp.PhysxArticulation: 
    
    success, obj_pose = rand_pose_messy(
        xlim=xlim,
        ylim=ylim,
        zlim=zlim,
        ylim_prop=ylim_prop,
        rotate_rand=rotate_rand,
        rotate_lim=rotate_lim,
        qpos=qpos,
        size_dict=size_dict,
        obj_radius = obj_radius,
        z_offset = z_offset,
        z_max = z_max,
        prohibited_area = prohibited_area
    )
    if not success:
        return False, None

    return True, create_messy_urdf_obj(
        scene,
        pose= obj_pose,
        modelname=modelname,
        scale=scale,
        fix_root_link = fix_root_link
    )