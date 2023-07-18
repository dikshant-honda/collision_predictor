import math

def overlap_area_of_circles(r1, r2, d):
    # Calculate the distance between the centers
    distance = math.sqrt((d[0] - d[1])**2 + (d[2] - d[3])**2)

    # Case 1: One circle is entirely within the other
    if distance < abs(r1 - r2):
        smaller_radius = min(r1, r2)
        return math.pi * smaller_radius**2

    # Case 2: The circles don't overlap at all
    if distance >= r1 + r2:
        return 0

    # Case 3: Partial overlap
    theta1 = 2 * math.acos((distance**2 + r1**2 - r2**2) / (2 * distance * r1))
    theta2 = 2 * math.acos((distance**2 + r2**2 - r1**2) / (2 * distance * r2))
    sector_area1 = (theta1 / 2) * r1**2
    sector_area2 = (theta2 / 2) * r2**2
    triangle_area = 0.5 * distance * math.sqrt(r1**2 - (distance**2 / 4)) * math.sqrt(r2**2 - (distance**2 / 4))
    overlap_area = sector_area1 + sector_area2 - triangle_area

    return overlap_area

# Example usage
radius1 = 5
radius2 = 3
center_distance = [0, 7.9, 0, 0]

area = overlap_area_of_circles(radius1, radius2, center_distance)
print("Overlap area:", area)
