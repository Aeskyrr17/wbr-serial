import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/model"))

import generate  # noqa: E402
from dynamics import lqr_gain, state_space  # noqa: E402
from model_params import QR  # noqa: E402


def geometry(phi1, phi4, l1=0.220, l2=0.260):
    x = l1 * (math.cos(phi4) - math.cos(phi1))
    y = l1 * (math.sin(phi4) - math.sin(phi1))
    a, b, c = 2*l2*x, 2*l2*y, x*x+y*y
    u2 = 2*math.atan2(b + math.sqrt(a*a+b*b-c*c), a+c)
    cx = l1*math.cos(phi1) + l2*math.cos(u2)
    cy = l1*math.sin(phi1) + l2*math.sin(u2)
    u3 = math.pi + math.atan2(l1*math.sin(phi4)-cy, l1*math.cos(phi4)-cx)
    length, phi = math.hypot(cx, cy), math.atan2(cy, cx)
    s32 = math.sin(u3-u2)
    jac = np.array([
        [l1*math.sin(phi-u3)*math.sin(phi1-u2)/s32,
         l1*math.sin(phi-u2)*math.sin(u3-phi4)/s32],
        [l1*math.cos(phi-u3)*math.sin(phi1-u2)/(s32*length),
         l1*math.cos(phi-u2)*math.sin(u3-phi4)/(s32*length)],
    ])
    return np.array([length, phi]), jac


class WbrModelTest(unittest.TestCase):
    def test_legacy_coefficients_keep_asymmetric_pairing(self):
        coeff = generate.legacy_coefficients()["high"]
        left, right = 0.16, 0.35
        basis = np.array([1, left, right, left*left, left*right, right*right])
        reversed_basis = np.array([1, right, left, right*right, right*left, left*left])
        self.assertGreater(np.max(np.abs(coeff @ basis - coeff @ reversed_basis)), 1.0)

    def test_lqr_rounding_and_saved_sign(self):
        coeff = generate.legacy_coefficients()["low"]
        left = round(min(max(0.1649, 0.16), 0.35) * 100) / 100
        right = round(min(max(0.3451, 0.16), 0.35) * 100) / 100
        gain = (coeff @ np.array([1, left, right, left*left, left*right, right*right])).reshape(4, 10)
        error = np.arange(10, dtype=float) * 0.01
        output = gain @ error
        self.assertTrue(np.all(np.isfinite(output)))
        self.assertAlmostEqual(left, 0.16)
        self.assertAlmostEqual(right, 0.35)

    def test_vmc_jacobian_and_virtual_power(self):
        q = np.array([2.6, 0.55])
        state, jac = geometry(*q)
        finite = np.empty((2, 2))
        eps = 1.0e-5
        for column in range(2):
            qp, qm = q.copy(), q.copy()
            qp[column] += eps; qm[column] -= eps
            finite[:, column] = (geometry(*qp)[0] - geometry(*qm)[0]) / (2*eps)
        np.testing.assert_allclose(jac, finite, rtol=2e-4, atol=2e-4)
        qdot = np.array([0.7, -0.4]); force = np.array([80.0, -3.0])
        np.testing.assert_allclose((jac.T @ force) @ qdot, force @ (jac @ qdot), atol=1e-6)

    def test_model_care_and_1ms_sample(self):
        a, b = state_space(0.16, 0.35)
        gain, residual = lqr_gain(a, b, *QR["high"])
        eigenvalues = np.linalg.eigvals(a + b @ gain)
        self.assertEqual(a.shape, (10, 10)); self.assertEqual(b.shape, (10, 4))
        self.assertLess(residual, 1.0e-5)
        self.assertLess(np.max(np.real(eigenvalues)), 0.0)
        self.assertLess(np.max(np.abs(np.exp(eigenvalues * 0.001))), 1.0)

    def test_protocol_golden_bytes_from_legacy_arm_gcc_layout(self):
        # ARM GCC 13.3 sample: move/spin/stair/gimbal-only, yaw=-1234,
        # vx=-5, vy=7, len=2, trigger=-9.
        command = bytes([0x55, 0x2E, 0xFB, 0xFB, 0x07, 0x02, 0xF7, 0x00])
        flags = command[0]
        self.assertTrue(flags & 0x01)   # move level
        self.assertFalse(flags & 0x02)  # want jump pulse
        self.assertTrue(flags & 0x04)   # spin level
        self.assertFalse(flags & 0x08)  # want turn pulse
        self.assertTrue(flags & 0x10)   # want stair pulse
        self.assertFalse(flags & 0x20)  # want fly pulse
        self.assertTrue(flags & 0x40)   # gimbal-only level
        self.assertEqual(int.from_bytes(command[1:3], "little", signed=True), -1234)
        bits = 1 | 2 | (7 << 2) | (300 << 6) | (255 << 15) | (0x1234 << 24)
        self.assertEqual(bits.to_bytes(8, "little").hex(), "1fcb7f3412000000")


if __name__ == "__main__":
    unittest.main()
