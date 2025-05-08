import h5py, pdb, os
from PIL import Image

def save_img_png(img, save_path):
    img = Image.fromarray(img)
    img.save(save_path)

def load_robotwin_hdf5_joint(dataset_path):
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    with h5py.File(dataset_path, 'r') as root:
        vector = root['/joint_action/vector'][()]

    return vector

def load_hdf5(dataset_path):
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    with h5py.File(dataset_path, 'r') as root:
        import pdb
        pdb.set_trace()
        left_gripper, left_arm = root['observations']['qpos'][:, :6], root['observations']['qpos'][:, 6]
        right_gripper, right_arm = root['observations']['qpos'][:, 7:13], root['observations']['qpos'][:, 13]
        image_dict = dict()
        for cam_name in root[f'observations/images'].keys():
            image_dict[cam_name] = root[f'observations/images/{cam_name}'][()]

    return left_gripper, left_arm, right_gripper, right_arm, image_dict

# file_path = 'real_data/czx/episode_1.hdf5'
# left_gripper, left_arm, right_gripper, right_arm, image_dict = load_hdf5(file_path)

vector = load_robotwin_hdf5_joint('/home/agilex/Desktop/AgileX-Piper-Deployment/RoboTwin_Data/dual_bottles_pick_easy/piper+piper-m0_b1_l1_h0_c0_D435/episode0.hdf5')


# <KeysViewHDF5 ['action', 'base_action', 'observations']>
# <HDF5 dataset "action": shape (250, 14), type "<f4">
# <HDF5 dataset "base_action": shape (250, 2), type "<f4">
# observations: 
#   -- images: cam_high, cam_left_wrist, cam_right_wrist
# <HDF5 dataset "cam_high": shape (250, 480, 640, 3), type "|u1">
# <HDF5 dataset "qpos": shape (250, 14), type "<f4">