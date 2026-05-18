import os
import hydra
import torch
from torch.utils.tensorboard import SummaryWriter
from omegaconf import DictConfig, OmegaConf
from loguru import logger
from utils.misc import compute_model_dim
from utils.io import mkdir_if_not_exists
from utils.plot import Ploter
from datasets.base import create_dataset
from datasets.misc import collate_fn_general, collate_fn_squeeze_pcd_batch
from models.base import create_model
from models.visualizer import create_visualizer
from tqdm import tqdm  
from functools import partial
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
    torch.save({
        'model': saved_state_dict,
        'epoch': epoch, 'step': step,
    }, path)

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
def train(cfg: DictConfig) -> None:
    """ training portal

    Args:
        cfg: configuration dict
    """
    if cfg.gpu is not None:
        device = f'cuda:{cfg.gpu}'
    else:
        device = 'cpu'

    ## prepare dataset for train and test
    datasets = {
        'train': create_dataset(cfg.task.dataset, 'train', cfg.slurm),
    }
    if cfg.task.visualizer.visualize:
        datasets['test_for_vis'] = create_dataset(cfg.task.dataset, 'test', cfg.slurm, case_only=True)
    for subset, dataset in datasets.items():
        logger.info(f'Load {subset} dataset size: {len(dataset)}')
    
    if cfg.model.scene_model.name == 'PointTransformer':
        collate_fn = partial(collate_fn_squeeze_pcd_batch, use_llm=cfg.model.use_llm)
    else:
        collate_fn = partial(collate_fn_general, use_llm=cfg.model.use_llm)
    
    dataloaders = {
        'train': datasets['train'].get_dataloader(
            batch_size=cfg.task.train.batch_size,
            collate_fn=collate_fn,
            num_workers=cfg.task.train.num_workers,
            pin_memory=True,
            shuffle=True,
        ),
    }
    
    ## create model and optimizer
    model = create_model(cfg, slurm=cfg.slurm, device=device)
    model.to(device=device)
    
    params = []
    nparams = []
    for n, p in model.named_parameters():
        if p.requires_grad:
            params.append(p)
            nparams.append(p.nelement())
    
    params_group = [
        {'params': params, 'lr': cfg.task.lr},
    ]
    optimizer = torch.optim.Adam(params_group) # use adam optimizer in default
    logger.info(f'{len(params)} parameters for optimization.')
    logger.info(f'total model size is {sum(nparams)}.')
    ## create visualizer if visualize in training process
    if cfg.task.visualizer.visualize:
        visualizer = create_visualizer(cfg.task.visualizer)
    start_epoch, start_step = load_ckpt(model, cfg.ckpt_dir, cfg.save_model_seperately)
    ## start training
    step = 0
    for epoch in range(0, cfg.task.train.num_epochs):
        model.train()
        
        progress_bar = tqdm(dataloaders['train'], desc=f'Epoch {epoch + 1}/{cfg.task.train.num_epochs}')
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
            ## plot loss
            if (step + 1) % cfg.task.train.log_step == 0:
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
        if (epoch + 1) % cfg.save_model_interval == 0:
            save_path = os.path.join(
                cfg.ckpt_dir, 
                f'model_{epoch}.pth' if cfg.save_model_seperately else 'model.pth'
            )

            save_ckpt(
                model=model, epoch=epoch, step=step, path=save_path,
                save_scene_model=cfg.save_scene_model,
            )



@hydra.main(version_base=None, config_path="./configs", config_name="default")
def main(cfg: DictConfig) -> None:
    ## compute modeling dimension according to task
    cfg.model.d_x = compute_model_dim(cfg.task)
    if os.environ.get('SLURM') is not None:
        cfg.slurm = True # update slurm config
        logger.remove(handler_id=0) # remove default handler

    ## set output logger and tensorboard
    logger.add(cfg.exp_dir + '/runtime.log')

    mkdir_if_not_exists(cfg.tb_dir)
    mkdir_if_not_exists(cfg.vis_dir)
    mkdir_if_not_exists(cfg.ckpt_dir)

    writer = SummaryWriter(log_dir=cfg.tb_dir)
    Ploter.setWriter(writer)

    ## Begin training progress
    logger.info('Configuration: \n' + OmegaConf.to_yaml(cfg))
    logger.info('Begin training..')

    train(cfg) # training portal

    ## Training is over!
    writer.close() # close summarywriter and flush all data to disk
    logger.info('End training..')

if __name__ == '__main__':
    main()
