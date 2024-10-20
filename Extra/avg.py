import csv
import numpy as np

def calculate_column_average(filename, column_index):
    total = 0
    count = 0

    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        header = next(reader)  # Skip the header row

        for row in reader:
            try:
                value = float(row[column_index])
                total += value
                count += 1
            except ValueError:
                # Handle non-numeric values (e.g., empty cells, text)
                pass

    if count == 0:
        return None  # No valid values found
    else:
        return total / count

A_x = calculate_column_average(r"/home/uday/Desktop/PerceptionFiles/A.csv", 1)
# print(A_x)
A_y = calculate_column_average(r"/home/uday/Desktop/PerceptionFiles/A.csv", 2)
# print(A_y)
A_z = calculate_column_average(r"/home/uday/Desktop/PerceptionFiles/A.csv", 3)
# print(A_z)
B_x = calculate_column_average(r"/home/uday/Desktop/PerceptionFiles/B.csv", 1)
# print(B_x)
B_y = calculate_column_average(r"/home/uday/Desktop/PerceptionFiles/B.csv", 2)
# print(B_y)
B_z = calculate_column_average(r"/home/uday/Desktop/PerceptionFiles/B.csv", 3)
# print(B_z)
C_x = calculate_column_average(r"/home/uday/Desktop/PerceptionFiles/C.csv", 1)
# print(C_x)
C_y = calculate_column_average(r"/home/uday/Desktop/PerceptionFiles/C.csv", 2)
# print(C_y)
C_z = calculate_column_average(r"/home/uday/Desktop/PerceptionFiles/C.csv", 3)
# print(C_z)


def find_plane_and_angle(p1, p2, p3):
    # Convert points to numpy arrays
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    
    # Compute vectors AB and AC
    AB = p2 - p1
    # print(AB)
    AC = p3 - p1
    # print(AC)
    
    # Compute the normal vector to the plane
    normal = np.cross(AB, AC)
    
    # Plane equation coefficients
    a, b, c = normal
    # print("normal", a,b,c,sep = ",")
    d = np.dot(normal, p1)  # Compute d in the plane equation
    
    # Plane equation: ax + by + cz = d
    plane_eq = f"{a}x + {b}y + {c}z = {d}"
    
    # Compute the angle with the xy-plane
    # Normal vector to the xy-plane is (0, 0, 1)
    normal_vector = np.array([0, 0, 1])
    
    # Dot product of the normal vector of the plane and the normal vector of the xy-plane
    dot_product = np.dot(normal, normal_vector)
    
    # Magnitude of the normal vector of the plane
    normal_magnitude = np.linalg.norm(normal)
    
    # Cosine of the angle
    # 0.9996969886526332
    cos_theta = c/(a**2 + b**2 + c**2)**0.5
    print("Cos", cos_theta)
    # Angle with the z-axis
    theta = np.arccos(cos_theta)
    print("Theta", theta)
    ang = 0.024618149859381887
    
    # Angle with the xy-plane
    angle_with_xy_plane = np.degrees(np.pi / 2 - theta)
    
    return plane_eq, angle_with_xy_plane

# p_a = B_y*C_z + B_z*C_x + B_x*C_y - (B_x*C_z + B_y*C_x + B_z*C_y)
# p_b = A_z*C_y + A_x*C_z + A_y*C_x - (A_z*C_x + A_x*C_y + A_y*C_z)
# p_c = A_y*B_z + A_x*B_y + A_z*B_x - (A_y*B_x + A_x*B_z + A_z*B_y)
# p_d = -A_x*p_a - B_x*p_b - C_x*p_c
# print(p_a, p_b,p_c, p_d, sep = " ")
# # Example points
p1 = (A_x, A_y, A_z)
p2 = (B_x, B_y, B_z)
p3 = (C_x, C_y, C_z)

# Calculate plane equation and angle
plane_eq, angle_with_xy_plane = find_plane_and_angle(p1, p2, p3)

print(f"Plane Equation: {plane_eq}")
# print(f"Angle with xy-plane: {angle_with_xy_plane:.2f} degrees")

# 1.644001886792453
# -0.7482377358490565
# -0.31502830188679254
# 4.413794736842106
# -2.8757210526315786
# -0.4004999999999999
# 7.370416666666667
# -2.6407583333333333
# -0.4597833333333334
# Plane Equation: 0.14620696524164203x + -0.08850494422707628y + 6.940961889675603z = -1.8800121717660754
# Angle with xy-plane: 88.59 degrees