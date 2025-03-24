from collections import deque
import torch
import random


class RoadmapReplayBuffer:
    def __init__(self, buffer_size, device):
        self.buffer = deque(maxlen=buffer_size)
        self.device = device
        self.sampled_data = {
            "node_inputs": None, "edge_inputs": None, "current_index": None,
            "node_padding_mask": None, "edge_padding_mask": None, "edge_mask": None,
            "action": None, "reward": None, "done": None,
            "next_node_inputs": None, "next_edge_inputs": None, "next_current_index": None,
            "next_node_padding_mask": None, "next_edge_padding_mask": None, "next_edge_mask": None
        }

    def add(self, roadmap_state, action, reward, done, next_roadmap_state):
        node_inputs, edge_inputs, current_index, node_padding_mask, edge_padding_mask, edge_mask = roadmap_state
        nx_node_inputs, nx_edge_inputs, nx_current_index, nx_node_padding_mask, nx_edge_padding_mask, nx_edge_mask = next_roadmap_state
        self.buffer.append(
            (node_inputs, edge_inputs, current_index,
             node_padding_mask, edge_padding_mask, edge_mask,
             action, reward, done,
             nx_node_inputs, nx_edge_inputs, nx_current_index,
             nx_node_padding_mask, nx_edge_padding_mask, nx_edge_mask))

    def sample(self, batch_size):
        node_inputs_batch, edge_inputs_batch, current_index_batch, node_padding_mask_batch, edge_padding_mask_batch, edge_mask_batch, action_batch, reward_batch, done_batch, nx_node_inputs_batch, nx_edge_inputs_batch, nx_current_index_batch, nx_node_padding_mask_batch, nx_edge_padding_mask_batch, nx_edge_mask_batch = zip(
            *random.sample(self.buffer, batch_size))

        self.sampled_data["node_inputs"] = torch.stack(node_inputs_batch).squeeze(1).to(self.device)  # (n_batchs, node_padding_size, n_features)
        self.sampled_data["edge_inputs"] = torch.stack(edge_inputs_batch).squeeze(1).to(self.device)   # (n_batchs, 1, k_neighbor_size)
        self.sampled_data["current_index"] = torch.stack(current_index_batch).squeeze(1).to(self.device)  # (n_batchs, 1, 1)
        self.sampled_data["node_padding_mask"] = torch.stack(node_padding_mask_batch).squeeze(1).to(self.device)  # (n_batchs, 1, node_padding_size)
        self.sampled_data["edge_padding_mask"] = torch.stack(edge_padding_mask_batch).squeeze(1).to(self.device)  # (n_batchs, 1, k_neighbor_size)
        self.sampled_data["edge_mask"] = torch.stack(edge_mask_batch).squeeze(1).to(self.device)  # (n_batchs, k_neighbor_size, k_neighbor_size)
        self.sampled_data["action"] = torch.stack(action_batch).squeeze(1).to(self.device)  # (n_batchs, 1, 1)
        self.sampled_data["reward"] = torch.stack(reward_batch).squeeze(1).to(self.device)  # (n_batchs, 1, 1)
        self.sampled_data["done"] = torch.stack(done_batch).squeeze(1).to(self.device)  # (n_batchs, 1, 1)
        self.sampled_data["next_node_inputs"] = torch.stack(nx_node_inputs_batch).squeeze(1).to(self.device)
        self.sampled_data["next_edge_inputs"] = torch.stack(nx_edge_inputs_batch).squeeze(1).to(self.device)
        self.sampled_data["next_current_index"] = torch.stack(nx_current_index_batch).squeeze(1).to(self.device)
        self.sampled_data["next_node_padding_mask"] = torch.stack(nx_node_padding_mask_batch).squeeze(1).to(self.device)
        self.sampled_data["next_edge_padding_mask"] = torch.stack(nx_edge_padding_mask_batch).squeeze(1).to(self.device)
        self.sampled_data["next_edge_mask"] = torch.stack(nx_edge_mask_batch).squeeze(1).to(self.device)

        return self.sampled_data