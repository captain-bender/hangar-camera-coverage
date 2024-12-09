# Required libraries
from shapely.geometry import Polygon, Point, box
import ezdxf
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

# Measure the pixel length of the aircraft
def measure_length(polygon):
    x, y = polygon.exterior.xy
    x_min, x_max = min(x), max(x)
    return x_max - x_min

# Known real-world length of the aircraft (in meters)
real_length_meters = 36.4478

# Function to generate intermediate points between two points
def interpolate_line(start, end, num_points=100):
    x = np.linspace(start[0], end[0], num_points)
    y = np.linspace(start[1], end[1], num_points)
    return list(zip(x, y))

def order_points(points):
    # Start with the first point
    print("Starting the ordering of the points")
    ordered = [points[0]]
    points = points[1:]
    
    while points:
        # Find the nearest point
        last = ordered[-1]
        distances = [np.linalg.norm(np.array(last) - np.array(p)) for p in points]
        nearest_index = np.argmin(distances)
        # Add it to the ordered list
        ordered.append(points.pop(nearest_index))
    
    # Close the loop
    if ordered[0] != ordered[-1]:
        ordered.append(ordered[0])
    print("Completed the ordering of the points")
    return ordered

# Function to group points into separate contours based on proximity
def group_points(points, threshold=50):
    groups = defaultdict(list)
    group_id = 0

    while points:
        if not groups[group_id]:
            groups[group_id].append(points.pop(0))
        current_point = groups[group_id][-1]
        distances = [np.linalg.norm(np.array(current_point) - np.array(p)) for p in points]
        if distances:
            nearest_index = np.argmin(distances)
            nearest_distance = distances[nearest_index]
            if nearest_distance < threshold:
                groups[group_id].append(points.pop(nearest_index))
            else:
                group_id += 1

    return list(groups.values())

# Load the DXF file
dxf_doc = ezdxf.readfile('737-400.dxf')
msp = dxf_doc.modelspace()

# Extract the boundary points
print("Extracting the boundary points")
boundary_points = []
for entity in msp:
    if entity.dxftype() == 'LWPOLYLINE':
        for point in entity.get_points():
            boundary_points.append((point[0], point[1]))
    elif entity.dxftype() == 'LINE':
        start_point = entity.dxf.start
        end_point = entity.dxf.end
        # Generate interpolated points for the line
        line_points = interpolate_line(start_point, end_point)
        boundary_points.extend(line_points)
    # Approximate the circle with points
    elif entity.dxftype() == 'SPLINE':
        for point in entity.control_points:
            boundary_points.append((point[0], point[1]))

boundary_points = list(dict.fromkeys(boundary_points))  # Remove duplicates

# Ensure the points form a closed loop
if boundary_points[0] != boundary_points[-1]:
    boundary_points.append(boundary_points[0])

# Group boundary points into separate contours
grouped_points = group_points(boundary_points, threshold=50)  # Adjust threshold if necessary

# Process each group of points to create polygons
print("Create polygons from the points")
polygons = []
for group in grouped_points:
    ordered = order_points(group)
    cleaned = list(dict.fromkeys(ordered))  # Remove duplicates
    if len(cleaned) > 2:  # Ensure valid polygon
        polygons.append(Polygon(cleaned))

# Calculate the scale factor
pixel_length = measure_length(polygons[0])  # Measure the aircraft length in pixels
scale_factor = real_length_meters / pixel_length

# Convert polygons to meters
scaled_polygons = []
for polygon in polygons:
    # Scale the coordinates
    scaled_exterior_coords = [(x * scale_factor, y * scale_factor) for x, y in polygon.exterior.coords]
    # Recreate the polygon with scaled coordinates
    scaled_polygon = Polygon(scaled_exterior_coords)
    scaled_polygons.append(scaled_polygon)

print("Creating the plots")
# Plot the polygons and their inflated versions
plt.figure(figsize=(10, 8))
for i, polygon in enumerate(scaled_polygons):
    if not polygon.is_empty:
        # Original polygon in meters
        x, y = polygon.exterior.xy
        plt.plot(x, y, linewidth=2, label=f'Original Contour {i+1}', color='blue')
        plt.fill(x, y, alpha=0.5, color='lightblue')

        # Inflated polygon in meters
        inflated_polygon = polygon.buffer(1.0)  # Inflate by 1 meter
        x_inflated, y_inflated = inflated_polygon.exterior.xy
        plt.plot(x_inflated, y_inflated, linewidth=2, label=f'Inflated Contour {i+1}', color='red')
        plt.fill(x_inflated, y_inflated, alpha=0.3, color='pink')

plt.xlabel('X Coordinate (meters)')
plt.ylabel('Y Coordinate (meters)')
plt.title('Aircraft Shape with Inflated Perimeter (in meters)')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show(block=False)

# Grid resolution in meters
grid_resolution = 0.5

# Get bounding box of the inflated polygon
minx, miny, maxx, maxy = inflated_polygon.bounds

# Generate grid points within the bounding box
x_coords = np.arange(minx, maxx + grid_resolution, grid_resolution)
y_coords = np.arange(miny, maxy + grid_resolution, grid_resolution)
xx, yy = np.meshgrid(x_coords, y_coords)
grid_points = np.vstack((xx.ravel(), yy.ravel())).T

# Filter points inside the inflated polygon
from shapely.geometry import Point

inside_points = []
for x, y in grid_points:
    point = Point(x, y)
    if inflated_polygon.contains(point):
        inside_points.append((x, y))
inside_points = np.array(inside_points)

# Plot the grid points inside the inflated perimeter in a separate figure
plt.figure(figsize=(10, 8))
inside_points = np.array(inside_points)  # Ensure it's a NumPy array
plt.scatter(inside_points[:, 0], inside_points[:, 1], color='green', s=10, label='Grid Points')

plt.xlabel('X Coordinate (meters)')
plt.ylabel('Y Coordinate (meters)')
plt.title('Grid Points Inside Inflated Perimeter')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()

# Known parameters
h = 12  # Hangar height in meters
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

# Overlap factor (less than or equal to 1 for closer spacing)
overlap_factor = 0.8

# Determine spacing for candidate camera positions
x_interval = W * overlap_factor
y_interval = L * overlap_factor

# Get bounding box of the polygon
minx, miny, maxx, maxy = inflated_polygon.bounds

# Generate candidate camera positions as a grid within the bounding box
camera_x_positions = np.arange(minx, maxx + x_interval, x_interval)
camera_y_positions = np.arange(miny, maxy + y_interval, y_interval)
camera_xx, camera_yy = np.meshgrid(camera_x_positions, camera_y_positions)
camera_positions = np.vstack((camera_xx.ravel(), camera_yy.ravel())).T

print("Number of candidate positions:", len(camera_positions))

# Assuming W, L, and camera_positions are already defined
camera_coverage_areas = []
for cam_pos in camera_positions:
    cam_x, cam_y = cam_pos
    half_W = W / 2
    half_L = L / 2
    # Create a rectangle centered at the camera position
    coverage_area = box(cam_x - half_W, cam_y - half_L, cam_x + half_W, cam_y + half_L)
    camera_coverage_areas.append(coverage_area)

# Initialize coverage matrix
num_points = inside_points.shape[0]
num_cameras = camera_positions.shape[0]
coverage_matrix = np.zeros((num_points, num_cameras), dtype=bool)

# Build coverage matrix
for cam_idx, coverage_area in enumerate(camera_coverage_areas):
    for pt_idx, (pt_x, pt_y) in enumerate(inside_points):
        if coverage_area.contains(Point(pt_x, pt_y)):
            coverage_matrix[pt_idx, cam_idx] = True