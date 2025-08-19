import numpy as np

def rotation_matrix_to_quaternion(R):
    """
    Converts a 3x3 rotation matrix to a quaternion (w, x, y, z).
    :param R: 3x3 numpy array representing a rotation matrix
    :return: np.array([w, x, y, z]) quaternion
    """
    assert R.shape == (3, 3), "Input must be a 3x3 matrix."
    m00, m01, m02 = R[0, :]
    m10, m11, m12 = R[1, :]
    m20, m21, m22 = R[2, :]

    tr = m00 + m11 + m22

    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2  # S=4*w
        w = 0.25 * S
        x = (m21 - m12) / S
        y = (m02 - m20) / S
        z = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2  # S=4*x
        w = (m21 - m12) / S
        x = 0.25 * S
        y = (m01 + m10) / S
        z = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2  # S=4*y
        w = (m02 - m20) / S
        x = (m01 + m10) / S
        y = 0.25 * S
        z = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2  # S=4*z
        w = (m10 - m01) / S
        x = (m02 + m20) / S
        y = (m12 + m21) / S
        z = 0.25 * S

    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)