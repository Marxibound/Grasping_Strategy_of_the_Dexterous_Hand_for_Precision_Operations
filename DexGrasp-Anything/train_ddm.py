import os
import hydra
import torch
import random
import numpy as np
from torch.utils.data.distributed import DistributedSampler
from torch.utils.tensorboard import SummaryWriter
from omegaconf import DictConfig, OmegaConf
from loguru import logger
import shutil
from utils.misc import compute_model_dim
from utils.io import mkdir_if_not_exists
from utils.plot import Ploter
from datasets.base import create_dataset
from datasets.misc import collate_fn_general, collate_fn_squeeze_pcd_batch
from models.base import create_model
from functools import partial
from tqdm import tqdm  
def save_ckpt(model: torch.nn.Module, epoch: int, step: int, path: str, save_scene_model: bool) -> None:
    """ Save current model and corresponding data

    Args:
        model: best model
        epoch: best epoch
        step: current step
        path: save path
        save_scene_model: if save scene_model
    """
    saved_state_dict = {}
    model_state_dict = model.state_dict()
    for key in model_state_dict:
        ## if use frozen pretrained scene model, we can avoid saving scene model to save space
        if 'scene_model' in key and not save_scene_model:
            continue

        saved_state_dict[key] = model_state_dict[key]
    
    logger.info('Saving model!!!' + ('[ALL]' if save_scene_model else '[Except SceneModel]'))
    
 # 管理检查点文件，只保留两个最新的检查点
    checkpoint_files = sorted([f for f in os.listdir(os.path.dirname(path)) if f.startswith('model_') and f.endswith('.pth')],
                              key=lambda x: int(x.split('_')[1].split('.')[0]))
    if len(checkpoint_files) >= 5:
        os.remove(os.path.join(os.path.dirname(path), checkpoint_files[0]))

    try:
        torch.save({
            'model': saved_state_dict,
            'epoch': epoch, 'step': step,
        }, path)
    except OSError as e:
        logger.error(f"Error saving model at {path}: {str(e)}")

def load_ckpt(model: torch.nn.Module, ckpt_dir: str, save_model_separately: bool) -> (int, int):
    """ Load model and corresponding data

    Args:
        model: model to load the state dict
        ckpt_dir: directory where checkpoints are saved
        save_model_separately: flag indicating if checkpoints are saved separately

    Returns:
        epoch: last epoch
        step: last step
    """
    if not os.path.exists(ckpt_dir):
        return 0, 0
    if save_model_separately:
        checkpoint_files = [f for f in os.listdir(ckpt_dir) if f.startswith('model_') and f.endswith('.pth')]
        if not checkpoint_files:
            return 0, 0

        latest_ckpt = max(checkpoint_files, key=lambda f: int(f.split('_')[1].replace('.pth', '')))
        ckpt_path = os.path.join(ckpt_dir, latest_ckpt)

    else:
        ckpt_path = os.path.join(ckpt_dir, 'model.pth')
        if not os.path.exists(ckpt_path):
            return 0, 0

    logger.info(f'Loading checkpoint from {ckpt_path}')
    checkpoint = torch.load(ckpt_path)
    epoch = checkpoint['epoch']
    step = checkpoint['step']
    saved_state_dict = checkpoint['model']
    model_state_dict = model.state_dict()

    for key in model_state_dict:
        if key in saved_state_dict:
            model_state_dict[key] = saved_state_dict[key]
        ## model is trained with ddm
        if 'module.'+key in saved_state_dict:
            model_state_dict[key] = saved_state_dict['module.'+key]
    model.load_state_dict(model_state_dict)
    return epoch, step

@hydra.main(version_base=None, config_path="./configs", config_name="default")
def main(cfg: DictConfig) -> None:
    """ training portal, train with multi gpus

    Args:
        cfg: configuration dict
    """
    ## set rank
    cfg.gpu = int(os.environ["LOCAL_RANK"])
    torch.cuda.set_device(cfg.gpu)
    device = torch.device('cuda', cfg.gpu)
    torch.distributed.init_process_group(backend='nccl')

    ## compute modeling dimension according to task
    cfg.model.d_x = compute_model_dim(cfg.task)
    if os.environ.get('SLURM') is not None:
        cfg.slurm = True # update slurm config
        logger.remove(handler_id=0) # remove default handler

    ## set output logger and tensorboard
    ## Begin training progress
    if cfg.gpu == 0:
        logger.add(cfg.exp_dir + '/runtime.log')

        mkdir_if_not_exists(cfg.tb_dir)
        mkdir_if_not_exists(cfg.ckpt_dir)

        writer = SummaryWriter(log_dir=cfg.tb_dir)
        Ploter.setWriter(writer)

        logger.info('Configuration: \n' + OmegaConf.to_yaml(cfg))
        logger.info('Begin training..')

    ## prepare dataset for train
    train_dataset = create_dataset(cfg.task.dataset, 'train', cfg.slurm)
    if cfg.gpu == 0:
        logger.info(f'Load train dataset size: {len(train_dataset)}')
    train_sampler = DistributedSampler(train_dataset,shuffle=True)

    if cfg.model.scene_model.name == 'PointTransformer':
        collate_fn = partial(collate_fn_squeeze_pcd_batch, use_llm=cfg.model.use_llm)
    else:
        collate_fn = partial(collate_fn_general, use_llm=cfg.model.use_llm)
    
    train_dataloader = train_dataset.get_dataloader(
        sampler=train_sampler,
        batch_size=cfg.task.train.batch_size,
        collate_fn=collate_fn,
        num_workers=cfg.task.train.num_workers,
        pin_memory=True
    )
    
    ## create model and optimizer
    model = create_model(cfg, slurm=cfg.slurm, device=device)
    model.to(device=device)

    params = []
    nparams = []
    for n, p in model.named_parameters():
        # 'TODO: add more parameters to freeze'
        # if 'eps_model.out_layers.0' not in n and 'eps_model.out_layers.2' not in n:
        #     p.requires_grad = False
        if p.requires_grad:
            params.append(p)
            nparams.append(p.nelement())

    
    params_group = [
        {'params': params, 'lr': cfg.task.lr},
    ]
    optimizer = torch.optim.Adam(params_group) # use adam optimizer in default
    if cfg.gpu == 0:
        logger.info(f'{len(params)} parameters for optimization.')
        logger.info(f'total model size is {sum(nparams)}.')
    ## convert to parallel
    model = torch.nn.parallel.DistributedDataParallel(
        model, device_ids=[cfg.gpu], output_device=cfg.gpu, find_unused_parameters=True)
    
    # Resume from checkpoint if exists
    start_epoch, start_step = load_ckpt(model, cfg.ckpt_dir, cfg.save_model_seperately)

    step = start_step
    for epoch in range(start_epoch, cfg.task.train.num_epochs):
        model.train()
        if epoch > start_epoch:
            start_step = 0
        progress_bar = tqdm(train_dataloader, desc=f'Epoch {epoch + 1}/{cfg.task.train.num_epochs}')
        for it, data in enumerate(progress_bar):
            for key in data:
                if torch.is_tensor(data[key]):
                    data[key] = data[key].to(device)
            optimizer.zero_grad()
            data['epoch'] = epoch
            outputs = model(data)
            outputs['loss'].backward()
            optimizer.step()
            total_loss = outputs['loss'].item()   
            progress_bar.set_postfix(loss=total_loss)  
            if cfg.gpu == 0 and (step + 1) % cfg.task.train.log_step == 0:
                log_str = f'[TRAIN] ==> Epoch: {epoch+1:3d} | Iter: {it+1:5d} | Step: {step+1:7d} | Loss: {total_loss:.3f}'
                logger.info(log_str)
                for key in outputs:
                    val = outputs[key].item() if torch.is_tensor(outputs[key]) else outputs[key]
                    Ploter.write({
                        f'train/{key}': {'plot': True, 'value': val, 'step': step},
                        'train/epoch': {'plot': True, 'value': epoch, 'step': step},
                    })
            step += 1
        ## save ckpt in epoch
        if cfg.gpu == 0 and (epoch + 1) % cfg.save_model_interval == 0:
            save_path = os.path.join(
                cfg.ckpt_dir, 
                f'model_{epoch}.pth' if cfg.save_model_seperately else 'model.pth'
            )
            
            save_ckpt(
                model=model, epoch=epoch, step=step, path=save_path,
                save_scene_model=cfg.save_scene_model,
            )

    ## Training is over!
    if cfg.gpu == 0:
        writer.close() # close summarywriter and flush all data to disk
        logger.info('End training..')

if __name__ == '__main__':
    ## set random seed
    torch.backends.cudnn.benchmark = False     
    torch.backends.cudnn.deterministic = True
    seed = 2022
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)

    main()
