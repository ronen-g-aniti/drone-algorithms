import matplotlib.pyplot as plt
import numpy as np

def plot_aabb_collision():
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Define the obstacle as a red rectangle
    obstacle_bottom_left = (2, 2)
    obstacle_width = 3
    obstacle_height = 3
    obstacle = plt.Rectangle(obstacle_bottom_left, obstacle_width, obstacle_height, color='red', alpha=0.5)
    ax.add_patch(obstacle)
    
    # Define the line segment
    line_start = np.array([1, 1])
    line_end = np.array([6, 4])
    ax.plot([line_start[0], line_end[0]], [line_start[1], line_end[1]], color='blue')
    
    # Define the bounding box for the line segment
    bbox_bottom_left = np.minimum(line_start, line_end)
    bbox_top_right = np.maximum(line_start, line_end)
    bbox_width = bbox_top_right[0] - bbox_bottom_left[0]
    bbox_height = bbox_top_right[1] - bbox_bottom_left[1]
    bbox = plt.Rectangle(bbox_bottom_left, bbox_width, bbox_height, edgecolor='blue', linestyle='--', linewidth=1, facecolor='none')
    ax.add_patch(bbox)
    
    # Check for intersection of bounding boxes
    obstacle_bbox = plt.Rectangle(obstacle_bottom_left, obstacle_width, obstacle_height, edgecolor='none', facecolor='none')
    
    # Find the intersection area
    intersect_x_min = max(obstacle_bottom_left[0], bbox_bottom_left[0])
    intersect_y_min = max(obstacle_bottom_left[1], bbox_bottom_left[1])
    intersect_x_max = min(obstacle_bottom_left[0] + obstacle_width, bbox_bottom_left[0] + bbox_width)
    intersect_y_max = min(obstacle_bottom_left[1] + obstacle_height, bbox_bottom_left[1] + bbox_height)
    
    if intersect_x_min < intersect_x_max and intersect_y_min < intersect_y_max:
        # There is an intersection
        intersection_width = intersect_x_max - intersect_x_min
        intersection_height = intersect_y_max - intersect_y_min
        intersection = plt.Rectangle((intersect_x_min, intersect_y_min), intersection_width, intersection_height, color='purple', alpha=0.7)
        ax.add_patch(intersection)
        ax.text(0.5, 0.9, 'Collision', transform=ax.transAxes, fontsize=14, color='purple', ha='center')
    else:
        # No intersection
        ax.text(0.5, 0.9, 'No Collision', transform=ax.transAxes, fontsize=14, color='green', ha='center')
    
    # Plot settings
    ax.set_xlim(0, 8)
    ax.set_ylim(0, 8)
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('AABB Collision Check')
    plt.grid(True)
    plt.show()

plot_aabb_collision()
