import math

def calculate_afov(sensor_dim, focal_length):
    return 2 * math.degrees(math.atan(sensor_dim / (2 * focal_length)))

# Example values
sensor_width = 6.4  # mm
sensor_height = 4.8  # mm
focal_length = 12    # mm

horizontal_afov = calculate_afov(sensor_width, focal_length)
vertical_afov = calculate_afov(sensor_height, focal_length)

print(f"Horizontal AFOV: {horizontal_afov:.2f} degrees")
print(f"Vertical AFOV: {vertical_afov:.2f} degrees")
