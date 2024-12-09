# Required libraries
import shapely
from shapely.geometry import Polygon
import ezdxf  # For reading DXF files
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

# Function to generate intermediate points between two points
def interpolate_line(start, end, num_points=100):
    x = np.linspace(start[0], end[0], num_points)
    y = np.linspace(start[1], end[1], num_points)
    return list(zip(x, y))

def order_points(points):
    # Start with the first point
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
polygons = []
for group in grouped_points:
    ordered = order_points(group)
    cleaned = list(dict.fromkeys(ordered))  # Remove duplicates
    if len(cleaned) > 2:  # Ensure valid polygon
        polygons.append(Polygon(cleaned))

# Plot the polygons
plt.figure(figsize=(10, 8))
for i, polygon in enumerate(polygons):
    if not polygon.is_empty:
        x, y = polygon.exterior.xy
        plt.plot(x, y, linewidth=2, label=f'Contour {i+1}')
        plt.fill(x, y, alpha=0.5)
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Ordered Aircraft Shape with Separate Contours')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()