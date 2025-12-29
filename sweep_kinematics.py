import math

def rot_x(ang_deg):
    a = math.radians(ang_deg)
    return [
        [1, 0, 0, 0],
        [0, math.cos(a), -math.sin(a), 0],
        [0, math.sin(a), math.cos(a), 0],
        [0, 0, 0, 1]
    ]

def rot_y(ang_deg):
    a = math.radians(ang_deg)
    return [
        [math.cos(a), 0, math.sin(a), 0],
        [0, 1, 0, 0],
        [-math.sin(a), 0, math.cos(a), 0],
        [0, 0, 0, 1]
    ]

def rot_z_correct(ang_deg):
    a = math.radians(ang_deg)
    c = math.cos(a)
    s = math.sin(a)
    return [
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]

def translate(x, y, z):
    return [
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ]

def mat_mul(A, B):
    C = [[0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            for k in range(4):
                C[i][j] += A[i][k] * B[k][j]
    return C

def rpy_to_mat(r, p, y):
    res = rot_z_correct(math.degrees(y))
    res = mat_mul(res, rot_y(math.degrees(p)))
    res = mat_mul(res, rot_x(math.degrees(r)))
    return res

def get_pose(j_angles, mount_rpy=[0, 0, 0], axes=[1, 1, 1, 1, 1, 1]):
    # Mount in james.urdf.xacro: xyz="0.15 0 0.05"
    T = translate(0.15, 0, 0.05)
    T = mat_mul(T, rpy_to_mat(mount_rpy[0], mount_rpy[1], mount_rpy[2]))
    
    # Joint 1: Stand upright (Pi on X)
    T = mat_mul(T, rpy_to_mat(math.pi, 0, 0))
    T = mat_mul(T, rot_z_correct(axes[0] * j_angles[0]))
    
    # Joint 2: origin rpy="1.5708 0 -1.5708" xyz="0 0.0642 -0.16977"
    T = mat_mul(T, translate(0, 0.0642, -0.16977))
    T = mat_mul(T, rpy_to_mat(1.5708, 0, -1.5708))
    T = mat_mul(T, rot_z_correct(axes[1] * j_angles[1]))
    
    # Joint 3: origin rpy="0 0 3.1416" xyz="0 -0.305 0.007"
    T = mat_mul(T, translate(0, -0.305, 0.007))
    T = mat_mul(T, rpy_to_mat(0, 0, 3.1416))
    T = mat_mul(T, rot_z_correct(axes[2] * j_angles[2]))
    
    # EE (Link 6 Center) - ignoring J4/J5 for now
    # Link 4 offset is 0 0 -0.22263
    # Link 5 offset is 0 0 0.041
    T = mat_mul(T, translate(0, 0, -0.22263))
    T = mat_mul(T, translate(0, 0, 0.041))
    
    return T[0][3], T[1][3], T[2][3]

# Test cases
# We want Y ~ 0.28, Z ~ 0.25 (elbow roughly)
j_angles = [0, 21.8, 85.0, 0, 0, 0]

print("--- TRYING MOUNT 3.14159, AXES J1=-1, J2=-1, J3=1 ---")
x, y, z = get_pose(j_angles, mount_rpy=[0, 0, 3.14159], axes=[-1, -1, 1])
# j2 = -21.8, j3 = 85.0
print(f"EE Pose: X={x:.4f}, Y={y:.4f}, Z={z:.4f}")

print("\n--- TRYING MOUNT 3.14159, AXES J1=-1, J2=1, J3=-1 ---")
x, y, z = get_pose(j_angles, mount_rpy=[0, 0, 3.14159], axes=[-1, 1, -1])
# j2 = 21.8, j3 = -85.0
print(f"EE Pose: X={x:.4f}, Y={y:.4f}, Z={z:.4f}")

print("\n--- TRYING MOUNT 0, AXES J1=1, J2=1, J3=-1 ---")
x, y, z = get_pose(j_angles, mount_rpy=[0, 0, 0], axes=[1, 1, -1])
print(f"EE Pose: X={x:.4f}, Y={y:.4f}, Z={z:.4f}")
