import unittest
import numpy as np

def compute_direction_vectors_vectorized(node_coords, target_positions, coords_norm_coef):
    """
    Compute normalized direction vectors and normalized distances from each node to each target
    in a fully vectorized manner, and return the output as a 2D array of shape (n_nodes, 3*n_targets).

    For each target, the output contains:
      [normalized_dx, normalized_dy, normalized_distance]
    and for multiple targets, these are concatenated along the last axis.

    Parameters:
        node_coords (np.ndarray): Array of shape (n_nodes, 2) with node (x, y) coordinates.
        target_positions (dict): Dictionary of target positions. Each value should be an iterable with two numbers.
        coords_norm_coef (float): A coefficient to normalize the distances.

    Returns:
        np.ndarray: An array of shape (n_nodes, 3*n_targets). For each node, the data is organized as:
                    [normalized_dx_target1, normalized_dy_target1, normalized_distance_target1,
                     normalized_dx_target2, normalized_dy_target2, normalized_distance_target2, ...]
    """
    # Ensure node_coords is a float array.
    node_coords = np.array(node_coords, dtype=np.float64)
    
    # Convert target positions to a float NumPy array of shape (n_targets, 2)
    targets = np.array(list(target_positions.values()), dtype=np.float64)  # shape: (n_targets, 2)
    
    # Reshape for broadcasting:
    #   node_coords: (n_nodes, 1, 2)
    #   targets:     (1, n_targets, 2)
    diff = targets[None, :, :] - node_coords[:, None, :]  # shape: (n_nodes, n_targets, 2)
    
    # Compute Euclidean distances along the last axis, keeping dimensions.
    dist = np.linalg.norm(diff, axis=2, keepdims=True)  # shape: (n_nodes, n_targets, 1)
    
    # Normalize the difference vectors safely (avoid division by zero).
    # Here we ensure the output array is float.
    normalized_diff = np.divide(diff, dist, out=np.zeros_like(diff, dtype=np.float64), where=(dist != 0))
    
    # Normalize distances using the coefficient.
    normalized_dist = dist / coords_norm_coef  # shape: (n_nodes, n_targets, 1)
    
    # Concatenate the normalized difference vectors and normalized distances along the last axis.
    # This yields an array of shape (n_nodes, n_targets, 3)
    concatenated = np.concatenate([normalized_diff, normalized_dist], axis=2)
    
    # Reshape to (n_nodes, 3*n_targets)
    n_nodes, n_targets, _ = concatenated.shape
    result = concatenated.reshape(n_nodes, n_targets * 3)
    
    return result



class TestComputeDirectionVectorsVectorized(unittest.TestCase):
    
    def test_single_node_multiple_targets(self):
        # Single node at (0, 0)
        node_coords = np.array([[0, 0]])
        
        # Define three targets
        target_positions = {
            'target1': [1, 0],   # To the right
            'target2': [0, 1],   # Upwards
            'target3': [-1, -1]  # Down-left
        }
        coords_norm_coef = 1
        
        # Compute the direction vectors.
        result = compute_direction_vectors_vectorized(node_coords, target_positions, coords_norm_coef)
        
        # Expected for node (0,0):
        # target1: diff = [1, 0] -> normalized = [1, 0], distance = 1.
        # target2: diff = [0, 1] -> normalized = [0, 1], distance = 1.
        # target3: diff = [-1, -1] -> norm = sqrt(2) -> normalized = [-1/sqrt(2), -1/sqrt(2)], distance = sqrt(2).
        sqrt2 = np.sqrt(2)
        expected = np.array([1.0, 0.0, 1.0, 
                             0.0, 1.0, 1.0, 
                             -1/sqrt2, -1/sqrt2, sqrt2])
        np.testing.assert_allclose(result[0], expected, atol=1e-7)
    
    def test_multiple_nodes_and_targets(self):
        # Two nodes
        node_coords = np.array([[0, 0], [3, 4]])
        
        # Two targets
        target_positions = {
            'A': [0, 0],  # Target at origin
            'B': [3, 4]   # Target coinciding with second node
        }
        coords_norm_coef = 2
        
        result = compute_direction_vectors_vectorized(node_coords, target_positions, coords_norm_coef)
        # Expected:
        # For node (0,0):
        #   To target A: diff = [0,0] -> normalized = [0,0], distance = 0.
        #   To target B: diff = [3,4] -> norm = 5 -> normalized = [0.6, 0.8], distance = 5/2 = 2.5.
        expected_node0 = np.array([0.0, 0.0, 0.0, 0.6, 0.8, 2.5])
        # For node (3,4):
        #   To target A: diff = [-3,-4] -> norm = 5 -> normalized = [-0.6, -0.8], distance = 5/2 = 2.5.
        #   To target B: diff = [0,0] -> normalized = [0,0], distance = 0.
        expected_node1 = np.array([-0.6, -0.8, 2.5, 0.0, 0.0, 0.0])
        
        np.testing.assert_allclose(result[0], expected_node0, atol=1e-7)
        np.testing.assert_allclose(result[1], expected_node1, atol=1e-7)

if __name__ == "__main__":
    unittest.main()
