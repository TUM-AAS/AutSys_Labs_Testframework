import numpy as np
import numpy.testing as npt

def test_intrinsic():
    intrinsics = np.loadtxt('param/intrinsics.txt')
    ref_intrinsics = np.array([
        [1000.0,   0.0, 640.0],
        [  0.0, 1000.0, 360.0],
        [  0.0,   0.0,   1.0]
    ])

    # Check closeness
    npt.assert_allclose(intrinsics, ref_intrinsics, atol=1, verbose=False, err_msg='Intrinsics are not close enough')

def test_extrinsic():
    extrinsics = np.loadtxt('param/extrinsics.txt')
    ref_extrinsics = np.array([
        [0.95, -0.14,  0.27, 0.06],
        [0.14,  0.99,  0.00, -0.01],
        [-0.27, 0.04,  0.96, 0.44],
        [0.00,  0.00,  0.00, 1.00]
    ])

    # Check closeness
    npt.assert_allclose(extrinsics, ref_extrinsics, atol=0.1, verbose=False, err_msg='Extrinsics are not close enough')

def test_distortion():
    distortion = np.loadtxt('param/distortion.txt')
    ref_distortion = np.array([
        0.0, 0.0, 0.0, 0.0, 0.0
    ])

    # Check closeness
    npt.assert_allclose(distortion, ref_distortion, atol=0.2, verbose=False, err_msg='Distortion is not close enough')
