import unittest

from lib import nmpc
from ._helpers import get_params_and_coeffs, get_state


class TestLibraryStructs(unittest.TestCase):

    def test_state(self):
        state = nmpc.State(0.5, 0.6, -0.001, 0.0, 0, 0, -0.93, -0.32)
        self.assertAlmostEqual(state.x, 0.5, 5)
        self.assertAlmostEqual(state.y, 0.6, 5)
        self.assertAlmostEqual(state.theta, -0.001, 5)
        self.assertAlmostEqual(state.v, 0.0000, 5)
        self.assertAlmostEqual(state.omega, 0, 5)
        self.assertAlmostEqual(state.throttle, 0, 5)
        self.assertAlmostEqual(state.cte, -0.93, 5)
        self.assertAlmostEqual(state.etheta, -0.32, 5)

    def test_params(self):
        params = nmpc.Params()
        params.forward.dt = 0.5
        self.assertAlmostEqual(params.forward.dt, 0.500, 5)


class TestSolver(unittest.TestCase):

    def test_solver1(self):
        params, coeffs = get_params_and_coeffs()
        state = get_state()
        coeffs = list(coeffs)

        mpc = nmpc.NMPC(params, coeffs)

        solution = mpc.solve(state)

        self.assertAlmostEqual(solution.omega, 0.503284, 6)
        self.assertEqual(solution.throttle, 1)
