import os
import tyro
from dataclasses import dataclass
from distutils.util import strtobool


@dataclass
class Args:
    # Preparing parameters
    exp_name: str = "None"
    """the name of this experiment, named by the executed python script"""
    seed: int = 1
    """seed of the experiment"""
    cuda: lambda x: bool(strtobool(x)) = True
    """if toggled, cuda will be enabled by default"""
    cuda_deterministic: lambda x: bool(strtobool(x)) = True
    """if toggled, `torch.backends.cudnn.deterministic=False`"""
    n_training_threads: int = 2
    """"number of torch threads for parallel CPU operations"""
    use_wandb: lambda x: bool(strtobool(x)) = True
    """if toggled, this experiment will be tracked with Weights and Biases"""
    wandb_project_name: str = "UAV-End2End-Navigation"
    """the wandb's project name"""
    wandb_entity: str = None
    """the entity (team) of wandb's project"""
    capture_video: lambda x: bool(strtobool(x)) = False
    """whether to capture videos of the agent performances (check out `videos` folder)"""
    save_model: lambda x: bool(strtobool(x)) = False
    """whether to save model into the `runs/{run_name}` folder"""
    upload_model: lambda x: bool(strtobool(x)) = False
    """whether to upload the saved model to huggingface"""

    # Algorithm specific parameters
    env_name: str = "UAV-End2End-Navigation"
    """the name of the environment"""
    total_timesteps: int = 1000000  # isaacgym: 30000000   mujoco: 1000000
    """total timesteps of the experiments"""
    num_episodes: int = 1000
    """total episodes of the experiments"""
    learning_rate: float = 1e-4  # isaacgym: 0.0026   mujoco: 3e-4
    """the learning rate of the optimizer"""
    num_envs: int = 1  # isaacgym: 4096   mujoco: 1
    """the number of parallel game environments"""
    num_steps: int = 512
    """the number of steps to run in each environment per policy rollout"""
    anneal_lr: lambda x: bool(strtobool(x)) = False
    """Toggle learning rate annealing for policy and value networks"""
    gamma: float = 0.99
    """the discount factor gamma"""
    gae_lambda: float = 0.95
    """the lambda for the general advantage estimation"""
    num_minibatches: int = 8
    """the number of mini-batches"""
    update_epochs: int = 10
    """the K epochs to update the policy"""
    norm_adv: lambda x: bool(strtobool(x)) = True
    """Toggles advantages normalization"""
    clip_coef: float = 0.2
    """the surrogate clipping coefficient"""
    clip_vloss: lambda x: bool(strtobool(x)) = False
    """Toggles whether or not to use a clipped loss for the value function, as per the paper."""
    ent_coef: float = 0
    """coefficient of the entropy"""
    vf_coef: float = 2
    """coefficient of the value function"""
    max_grad_norm: float = 1
    """the maximum norm for the gradient clipping"""
    target_kl: float = None
    """the target KL divergence threshold"""

    # UAV end-to-end navigation specific parameters
    step_time: float = 0.05
    """step time duration for executing a command from policy network"""
    linear_spd_limit_x: float = 2.0
    """forward linear speed limit of UAV"""
    linear_spd_limit_y: float = 0.05
    """left/right linear speed limit of UAV"""
    angular_spd_limit: float = 1.5
    """left/right angular speed limit of UAV"""


    # to be filled in runtime
    batch_size: int = 0
    """the batch size (computed in runtime)"""
    minibatch_size: int = 0
    """the mini-batch size (computed in runtime)"""
    num_iterations: int = 0
    """the number of iterations (computed in runtime)"""


def get_config():
    args = tyro.cli(Args)
    args.batch_size = int(args.num_envs * args.num_steps)
    args.minibatch_size = int(args.batch_size // args.num_minibatches)
    args.num_iterations = args.total_timesteps // args.batch_size

    return args
