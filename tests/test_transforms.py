import numpy as np

from piperx_grasp.calibration.transforms import matrix_to_xyz_rpy, xyz_rpy_to_matrix


def test_pose_matrix_round_trip_translation():
    pose = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
    matrix = xyz_rpy_to_matrix(pose)
    round_trip = matrix_to_xyz_rpy(matrix)
    assert np.allclose(round_trip[:3], pose[:3])
