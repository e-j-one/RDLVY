import numpy as np

def select_hightway_positions(highways, n):
    # Get the indices where gridmap is 1
    valid_indices = np.argwhere(highways == 1)
    
    # If not enough valid points are available, raise an error
    if valid_indices.shape[0] < n:
        raise ValueError(f'Only {valid_indices.shape[0]} valid points available, but {n} were requested.')

    # Select n random indices without replacement
    selected_indices = valid_indices[np.random.choice(valid_indices.shape[0], n, replace=False)]
    
    # Split the selected indices into x and y coordinates
    x_indices, y_indices = zip(*selected_indices)
    
    return np.array(x_indices), np.array(y_indices)