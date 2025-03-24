from os import path as os_path
import tyro
from dataclasses import dataclass
from distutils.util import strtobool


@dataclass
class FrontierGuidedDRMArgs:
    # Preparing parameters
    exp_name: str = "None"
    """the name of this experiment, named by the executed python script"""
    seed: int = 2
    """seed of the experiment"""
    cuda: lambda x: bool(strtobool(x)) = True
    """if toggled, cuda will be enabled by default"""
    cuda_deterministic: lambda x: bool(strtobool(x)) = True
    """if toggled, `torch.backends.cudnn.deterministic=False`"""
    n_training_threads: int = 2
    """"number of torch threads for parallel CPU operations"""
    use_wandb: lambda x: bool(strtobool(x)) = True
    """if toggled, this experiment will be tracked with Weights and Biases"""
    wandb_project_name: str = "nav-fg-drm"
    """the wandb's project name"""
    wandb_entity: str = None
    """the entity (team) of wandb's project"""
    capture_video: lambda x: bool(strtobool(x)) = False
    """whether to capture videos of the agent performances (check out `videos` folder)"""
    save_model: lambda x: bool(strtobool(x)) = True
    """whether to save model into the `runs/{run_name}` folder"""
    current_dir = os_path.dirname(os_path.abspath(__file__))
    model_path: str = current_dir + "/model_weights"
    """path to the weights of policy network"""
    map_name: str = "maze_medium"
    """the name of the environment map"""

    # Algorithm specific parameters
    env_name: str = "Nav-Frontier-guided-DRM"
    """the name of the environment"""
    device: str = "cpu"
    """the GPU device"""
    train_num_episodes: int = 1000
    """total episodes of the training experiments"""
    eval_num_episodes: int = 40
    """total episodes of the evaluation experiments"""
    max_episode_steps: int = 135
    """the number of steps in one episode"""
    buffer_size: int = int(1e4)
    """the replay memory buffer size"""
    gamma: float = 1
    """the discount factor gamma"""
    tau: float = 0.005
    """target smoothing coefficient"""
    batch_size: int = 128  
    """the batch size of sample from the reply memory"""
    learning_starts: int = 600
    """timestep to start learning"""
    policy_lr: float = 1e-5
    """the learning rate of the policy network optimizer"""
    q_lr: float = 2e-5
    """the learning rate of the Q network network optimizer"""
    lr_decay_step: float = 256
    """the step interval to perform a decay on learning rates"""
    policy_frequency: int = 2
    """the frequency of training policy (delayed)"""
    target_network_frequency: int = 1  # Denis Yarats' implementation delays this by 2.
    """the frequency of updates for the target networks"""
    model_save_frequency: int = 500
    """the frequency of saving model weights"""
    noise_clip: float = 0.5
    """noise clip parameter of the Target Policy Smoothing Regularization"""
    alpha: float = 0.2
    """Entropy regularization coefficient."""
    autotune: bool = True
    """automatic tuning of the entropy coefficient"""
    greedy: lambda x: bool(strtobool(x)) = True
    """whether to use greedy sample mechanism from policy output (only for eval-stage)"""
    input_dim: int = 7  # 2 + 1 + 1 + 3
    """input dimension of the attention network"""
    embedding_dim: int = 256
    """embedding dimension of the attention network"""

    # Roadmap env specific parameters
    k_neighbor_size: int = 41  # max_goal_candidates + 1
    """number of k-nearest neighbors for edge connection (including self-node), need to align with max_goal_candidates in autonomous_flight/cfg/fg_drm"""
    step_time: float = 0.8
    """step time duration for executing a command from policy network"""
    coords_norm_coef: float = 30.
    """coefficient for normalizing node coordinates"""
    node_padding_size: int = 41  # max_goal_candidates + 1
    """"the node number in the graph will be padded to node_padding_size during training to keep the consistency of training input tensor"""
    goal_reward: float = 20
    """goal-reach reward"""
    collision_reward: float = -20
    """collision reward"""
    step_penalty_reward: float = -0.5
    """penalty reward per step"""
    goal_dis_amp: float = 1/256. # maze_simple: 1/160. 
    """amplifier of goal approaching reward"""


def get_config_fg_drm(parse_cli_args=False):
    if parse_cli_args:
        return tyro.cli(FrontierGuidedDRMArgs)
    else:
        return FrontierGuidedDRMArgs()

