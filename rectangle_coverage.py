import numpy as np

# Known parameters
h = 20  # Hangar height in meters
theta_h = 60  # Horizontal FOV in degrees
theta_v = 45  # Vertical FOV in degrees

# Convert FOV angles to radians
theta_h_rad = np.radians(theta_h)
theta_v_rad = np.radians(theta_v)

# Calculate coverage dimensions
W = 2 * h * np.tan(theta_h_rad / 2)
L = 2 * h * np.tan(theta_v_rad / 2)

print(f"Coverage Width (W): {W:.2f} meters")
print(f"Coverage Length (L): {L:.2f} meters")

# Aircraft dimensions
aircraft_length = 70  # meters
aircraft_width = 60   # meters

# Grid resolution in meters
grid_resolution = 1

# Generate grid points
x_coords = np.arange(0, aircraft_length + grid_resolution, grid_resolution)
y_coords = np.arange(0, aircraft_width + grid_resolution, grid_resolution)
xx, yy = np.meshgrid(x_coords, y_coords)
grid_points = np.vstack((xx.ravel(), yy.ravel())).T

# Overlap factor (0 < overlap_factor <= 1)
overlap_factor = 0.8

# Calculate intervals between cameras
x_interval = W * overlap_factor
y_interval = L * overlap_factor

# Generate camera positions
camera_x_positions = np.arange(0, aircraft_length + x_interval, x_interval)
camera_y_positions = np.arange(0, aircraft_width + y_interval, y_interval)
camera_xx, camera_yy = np.meshgrid(camera_x_positions, camera_y_positions)
camera_positions = np.vstack((camera_xx.ravel(), camera_yy.ravel())).T

from shapely.geometry import Point, box

# Initialize coverage matrix
num_points = grid_points.shape[0]
num_cameras = camera_positions.shape[0]
coverage_matrix = np.zeros((num_points, num_cameras), dtype=bool)

# Precompute camera coverage areas
camera_coverage_areas = []
for cam_pos in camera_positions:
    cam_x, cam_y = cam_pos
    half_W = W / 2
    half_L = L / 2
    coverage_area = box(cam_x - half_W, cam_y - half_L, cam_x + half_W, cam_y + half_L)
    camera_coverage_areas.append(coverage_area)

# Build coverage matrix
for cam_idx, coverage_area in enumerate(camera_coverage_areas):
    for pt_idx, (pt_x, pt_y) in enumerate(grid_points):
        if coverage_area.contains(Point(pt_x, pt_y)):
            coverage_matrix[pt_idx, cam_idx] = True

# Using PuLP
from pulp import LpProblem, LpMinimize, LpVariable, LpBinary, lpSum, LpStatus

# Define the problem
prob = LpProblem("CameraCoverageOptimization", LpMinimize)

# Decision variables
camera_vars = LpVariable.dicts("Camera", range(num_cameras), cat=LpBinary)

# Objective function
prob += lpSum([camera_vars[i] for i in range(num_cameras)])

# Constraints
for pt_idx in range(num_points):
    prob += lpSum([coverage_matrix[pt_idx, cam_idx] * camera_vars[cam_idx] for cam_idx in range(num_cameras)]) >= 1, f"Point_{pt_idx}_coverage"

# Solve the problem
prob.solve()

print(f"Optimization Status: {LpStatus[prob.status]}")
selected_cameras = [cam_idx for cam_idx in range(num_cameras) if camera_vars[cam_idx].varValue == 1]
print(f"Number of cameras selected: {len(selected_cameras)}")

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# Create a plot
fig, ax = plt.subplots(figsize=(12, 8))

# Plot aircraft outline
aircraft_rect = Rectangle((0, 0), aircraft_length, aircraft_width, linewidth=1, edgecolor='black', facecolor='none')
ax.add_patch(aircraft_rect)

# Plot selected cameras and their coverage areas
for cam_idx in selected_cameras:
    cam_x, cam_y = camera_positions[cam_idx]
    # Plot camera position
    ax.plot(cam_x, cam_y, 'ro')  # Red dot for camera
    # Plot coverage area
    half_W = W / 2
    half_L = L / 2
    coverage_rect = Rectangle((cam_x - half_W, cam_y - half_L), W, L, linewidth=1, edgecolor='blue', facecolor='blue', alpha=0.2)
    ax.add_patch(coverage_rect)

# Set plot limits
ax.set_xlim(-10, aircraft_length + 10)
ax.set_ylim(-10, aircraft_width + 10)
ax.set_aspect('equal')
ax.set_xlabel('Length (m)')
ax.set_ylabel('Width (m)')
ax.set_title('Optimized Camera Placement and Coverage')
plt.show()


