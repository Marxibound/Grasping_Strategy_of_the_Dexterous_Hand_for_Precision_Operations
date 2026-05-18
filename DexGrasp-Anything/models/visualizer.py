import os
import json
import torch
import torch.nn as nn
import numpy as np
from einops import rearrange
import trimesh
import pickle
from omegaconf import DictConfig
from plotly import graph_objects as go
from typing import Any
import random
from utils.registry import Registry
from utils.handmodel import get_handmodel
from utils.plotly_utils import plot_mesh
from utils.rot6d import rot_to_orthod6d, robust_compute_rotation_matrix_from_ortho6d, random_rot  
from tqdm import tqdm

VISUALIZER = Registry('Visualizer')
@VISUALIZER.register()
@torch.no_grad()
class GraspGenURVisualizer():
    def __init__(self, cfg: DictConfig) -> None:
        """ Visual evaluation class for pose generation task.
        Args:
            cfg: visuzalizer configuration
        """
        self.ksample = cfg.ksample
        self.hand_model = get_handmodel(batch_size=1, device='cuda')
        self.use_llm = cfg.use_llm
        self.visualize_html = cfg.visualize_html
        self.datasetname = cfg.datasetname
        ##############################################################################################################################
        ##### Since other datasets have object scale=1, for testing convenience we use average scales from DexGraspNet and UniDexGrasp test sets #####
        ##### To test with different mesh sizes, please adjust the mesh scale accordingly for proper visualization #####
        ##############################################################################################################################
        ### For DexGraspNet ###
        if self.datasetname == 'DexGraspNet':
            self.average_scales = self.load_average_scales('/inspurfs/group/mayuexin/datasets/DexGraspNet/scales.pkl')
        ### For UniDexGrasp ###
        else:
            self.average_scales = self.load_average_scales( '/inspurfs/group/mayuexin/datasets/UniDexGrasp/DFCData/scales.pkl')
        ##############################################################################################################################
        ##############################################################################################################################
    def load_average_scales(self, file_path):
        with open(file_path, 'rb') as f:
            return pickle.load(f)
    def visualize(
            self,
            model: torch.nn.Module,
            dataloader: torch.utils.data.DataLoader,
            save_dir: str
    ) -> None:
        """ Visualize method
        Args:
            model: diffusion model
            dataloader: test dataloader
            save_dir: save directory of rendering images
        """
        model.eval()
        device = model.device

        os.makedirs(save_dir, exist_ok=True)
        os.makedirs(os.path.join(save_dir, 'html'), exist_ok=True)
        # Getting descriptions from LLM
        if self.use_llm:
            self.scene_text = dataloader.dataset.scene_text
        # save
        pbar = tqdm(total=len(dataloader.dataset.split) * self.ksample)
        object_pcds_dict = dataloader.dataset.scene_pcds
        res = {'method': 'DGA@w/o-opt',
               'desc': 'w/o Physics-Guided Sampling',
               'sample_qpos': {}}
        # Define dataset configuration dictionary
        DATASET_CONFIG = {
            "DexGraspNet": {
                "scale_op": lambda x, s: x * s,
                "objects": dataloader.dataset._test_split
            },
            "Unidexgrasp": {
                "scale_op": lambda x, s: x / s,
                "objects": dataloader.dataset._test_split
            },
            "default": {
                "scale_op": lambda x, s: x,
                "objects": dataloader.dataset._test_split
            }
        }

        # Get current dataset configuration
        cfg = DATASET_CONFIG.get(
            dataloader.dataset.datasetname, 
            DATASET_CONFIG["default"]
        )

        # Unified processing flow
        for object_name in cfg["objects"]:
            # Point cloud scaling processing
            scale = self.average_scales.get(object_name, 1.0)  # Get average scale for object, default to 1.0
            obj_pcd_can = cfg["scale_op"](
                torch.tensor(object_pcds_dict[object_name], device=device).unsqueeze(0).repeat(self.ksample, 1, 1),
                scale
            )
            obj_pcd_nor = obj_pcd_can[:, :dataloader.dataset.num_points, 3:]
            obj_pcd_can = obj_pcd_can[:, :dataloader.dataset.num_points, :3]
            i_rot_list = []
            for k_rot in range(self.ksample):
                i_rot_list.append(random_rot(device))
            i_rot = torch.stack(i_rot_list).to(torch.float64)
            obj_pcd_rot = torch.matmul(i_rot, obj_pcd_can.transpose(1, 2)).transpose(1, 2)
            obj_pcd_nor_rot = torch.matmul(i_rot, obj_pcd_nor.transpose(1, 2)).transpose(1, 2)
            
            all_sentence = []
            for n in range(self.ksample):
                if self.use_llm:
                    all_sentence.extend(self.scene_text[object_name])               
                
            # construct data
            data = {'x': torch.randn(self.ksample, 27, device=device),
                    'pos': obj_pcd_rot.to(device),
                    'normal':obj_pcd_nor_rot.to(device),
                    'feat':obj_pcd_nor_rot.to(device),
                    'scene_rot_mat': i_rot,
                    'scene_id': [object_name for i in range(self.ksample)],
                    'cam_trans': [None for i in range(self.ksample)],
                    'text': (all_sentence if self.use_llm else None),
                    'sentence_cnt': ([len(self.scene_text[object_name])] * self.ksample if self.use_llm else None)}
            offset, count = [], 0
            for item in data['pos']:
                count += item.shape[0]
                offset.append(count)
            offset = torch.IntTensor(offset)
            data['offset'] = offset.to(device)
            data['pos'] = rearrange(data['pos'], 'b n c -> (b n) c').to(device)
            data['feat'] = rearrange(data['feat'], 'b n c -> (b n) c').to(device)
            outputs = model.sample(data, k=1).squeeze(1)[:, -1, :].to(torch.float64)
            
            ## denormalization
            if dataloader.dataset.normalize_x:
                outputs[:, 3:] = dataloader.dataset.angle_denormalize(joint_angle=outputs[:, 3:].cpu()).cuda()
            if dataloader.dataset.normalize_x_trans:
                outputs[:, :3] = dataloader.dataset.trans_denormalize(global_trans=outputs[:, :3].cpu()).cuda()
            id_6d_rot = torch.tensor([1., 0., 0., 0., 1., 0.], device=device).view(1, 6).repeat(self.ksample, 1).to(torch.float64)
            outputs_3d_rot = rot_to_orthod6d(torch.bmm(i_rot.transpose(1, 2), robust_compute_rotation_matrix_from_ortho6d(id_6d_rot)))
            outputs[:, :3] = torch.bmm(i_rot.transpose(1, 2), outputs[:, :3].unsqueeze(-1)).squeeze(-1)
            outputs = torch.cat([outputs[:, :3], outputs_3d_rot, outputs[:, 3:]], dim=-1)

            # visualization for checking
            scene_id = data['scene_id'][0]
            if dataloader.dataset.datasetname == 'MultiDexShadowHandUR' and self.visualize_html:
                scene_dataset, scene_object = scene_id.split('+')
                mesh_path = os.path.join(dataloader.dataset.asset_dir,'object', scene_dataset, scene_object, f'{scene_object}.stl')
                obj_mesh = trimesh.load(mesh_path)

            elif dataloader.dataset.datasetname == 'real_dex'and self.visualize_html:
                scene_object = scene_id
                mesh_path = os.path.join(dataloader.dataset.asset_dir,'meshdata', f'{scene_object}.obj')
                obj_mesh = trimesh.load(mesh_path)

            elif dataloader.dataset.datasetname == 'Unidexgrasp'and self.visualize_html:
                scene_object = scene_id
                mesh_path = os.path.join(dataloader.dataset.asset_dir,'obj_scale_urdf', f'{scene_object}.obj')
                obj_mesh = trimesh.load(mesh_path)

            elif dataloader.dataset.datasetname == 'DexGraspNet'and self.visualize_html:
                scene_object = scene_id
                mesh_path = os.path.join(dataloader.dataset.asset_dir,'obj_scale_urdf', f'{scene_object}.obj')
                obj_mesh = trimesh.load(mesh_path)

            elif dataloader.dataset.datasetname == 'DexGRAB'and self.visualize_html:
                scene_object = scene_id
                mesh_path = os.path.join(dataloader.dataset.asset_dir,'contact_meshes', f'{scene_object}.ply')
            
            elif dataloader.dataset.datasetname == 'Grasp_anyting'and self.visualize_html:
                scene_object = scene_id
                mesh_path = os.path.join(dataloader.dataset.asset_dir,'meshdata', f'{scene_object}.obj')
                obj_mesh = trimesh.load(mesh_path)

            for i in range(outputs.shape[0]):
                if self.visualize_html:
                    self.hand_model.update_kinematics(q=outputs[i:i+1, :])
                    vis_data = [plot_mesh(obj_mesh, color='lightpink')]
                    
                    vis_data += self.hand_model.get_plotly_data(opacity=1.0, color='#8799C6')
                    # Save as HTML file
                    save_path = os.path.join(save_dir, 'html', f'{object_name}+sample-{i}.html')
                    fig = go.Figure(data=vis_data)
                    fig.update_layout(
                        scene=dict(
                            xaxis=dict(visible=False),
                            yaxis=dict(visible=False),
                            zaxis=dict(visible=False),
                            bgcolor="white"
                        )
                    )
                    fig.write_html(save_path)
                pbar.update(1)
            res['sample_qpos'][object_name] = np.array(outputs.cpu().detach())
        pickle.dump(res, open(os.path.join(save_dir, 'res_diffuser.pkl'), 'wb'))

def create_visualizer(cfg: DictConfig) -> nn.Module:
    """ Create a visualizer for visual evaluation
    Args:
        cfg: configuration object
        slurm: on slurm platform or not. This field is used to specify the data path
    
    Return:
        A visualizer
    """
    return VISUALIZER.get(cfg.name)(cfg)

