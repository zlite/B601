"""Rotation vectors that retain small finite-difference rotations."""
from scipy.spatial.transform import Rotation

def rotation_vector(matrix):
    return Rotation.from_matrix(matrix).as_rotvec()
