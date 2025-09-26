import unittest

from pronav import ProNavType, State, ProNav

class TestProNav(unittest.TestCase):
    def setUp(self):
        self.pronav = ProNav()

    def test_state_to_ned(self):
        i = State(0, 50, 30, 1000, 0, 0, 0)
        t = State(0, 50.1, 30.1, 1100, 0, 0, 0)

        range_it = self.pronav.state_to_ned(i, t)

        self.assertAlmostEqual(range_it[0], 11121, 0)
        self.assertAlmostEqual(range_it[1], 7141, 0)
        self.assertAlmostEqual(range_it[2], -100, 0)

        range_ti = self.pronav.state_to_ned(t, i)

        self.assertAlmostEqual(range_ti[0], -11121, 0)
        self.assertAlmostEqual(range_ti[1], -7141, 0)
        self.assertAlmostEqual(range_ti[2], 100, 0)

        range_ii = self.pronav.state_to_ned(i, i)

        self.assertAlmostEqual(range_ii[0], 0, 0)
        self.assertAlmostEqual(range_ii[1], 0, 0)
        self.assertAlmostEqual(range_ii[2], 0, 0)

        # equator crossing
        i = State(0, -0.1, 0, 1000, 0, 0, 0)
        t = State(0, 0.1, 0, 1000, 0, 0, 0)

        range_it = self.pronav.state_to_ned(i, t)
        self.assertAlmostEqual(range_it[0], 22242, 0)
        self.assertAlmostEqual(range_it[1], 0, 0)
        self.assertAlmostEqual(range_it[2], 0, 0)

    def test_state_to_ctx(self):
        i = State(0, 50, 30, 1000, 10, 20, 5)
        t = State(0, 50.1, 30.1, 1100, -2, 10, -4)

        ctx = self.pronav.state_to_ctx(i, t)

        self.assertAlmostEqual(ctx.range_norm, 13217.06, 1)
        self.assertAlmostEqual(ctx.vel_i_norm, 22.9, 1)
        self.assertAlmostEqual(ctx.vel_t_norm, 11, 1)
        self.assertAlmostEqual(ctx.vel_rel_norm, 18, 1)
        self.assertAlmostEqual(ctx.vel_closing, 15.6, 1)

    def test_guidance_type(self):
        i = State(0, 50, 30, 1000, -10, -20, -5)
        t1 = State(0, 50 + .1, 30 + .1, 1100, 2, 10, 4)
        t2 = State(0, 50 - .1, 30 - .1, 1100, 2, 10, 4)

        command = self.pronav.compute_guidance(i, t1)
        self.assertEqual(command.nav_type, ProNavType.PURSUIT)

        command = self.pronav.compute_guidance(i, t2)
        self.assertEqual(command.nav_type, ProNavType.TPN)

    def test_compute_tail_bias(self):
        i = State(0, 50, 30, 1000, 10, 0, 0)
        t = State(0, 50.1, 30, 1000, 5, 0, 0)
        ctx = self.pronav.state_to_ctx(i, t)

        a_bias = self.pronav.compute_tail_bias(ctx)

        self.assertEqual(a_bias[0], 0)
        self.assertEqual(a_bias[1], 0)
        self.assertEqual(a_bias[2], 0)

        t.vel_north = 0
        t.vel_east = 5
        ctx = self.pronav.state_to_ctx(i, t)

        a_bias = self.pronav.compute_tail_bias(ctx)

        self.assertGreater(a_bias[0], 0)
        self.assertLess(a_bias[1], 0)
        self.assertEqual(a_bias[2], 0)

    def test_compute_pursuit(self):
        i = State(0, 50, 30, 1000, -10, 5, 0)
        t = State(0, 50.1, 30, 1000, 10, 0, 0)
        ctx = self.pronav.state_to_ctx(i, t)

        a = self.pronav.compute_guidance_pursuit(ctx)

        self.assertGreater(a[0], 0)
        self.assertGreater(a[1], 0)

        t, a = self.pronav.compute_guidance_accel(ctx)

        self.assertEqual(t, ProNavType.PURSUIT)

    def test_compute_pronav(self):
        i = State(0, 50, 30, 1000, 10, 0, 0)
        t = State(0, 50.1, 30, 1000, 0, 5, 0)
        ctx = self.pronav.state_to_ctx(i, t)

        a = self.pronav.compute_guidance_pronav(ctx)

        self.assertEqual(a[0], 0)
        self.assertGreater(a[1], 0)
        self.assertEqual(a[2], 0)

        t, a = self.pronav.compute_guidance_accel(ctx)

        self.assertEqual(t, ProNavType.TPN)

    def test_compute_tgo_zem(self):
        i = State(0, 50, 30, 1000, 5, 5, 0)
        t = State(0, 50.1, 30, 1000, 0, 5, 0)
        ctx = self.pronav.state_to_ctx(i, t)

        tgo, zem = self.pronav.compute_tgo_zem(ctx)

        self.assertAlmostEqual(tgo, 2224.2, 1)
        self.assertEqual(zem, 0)

if __name__ == '__main__':
    unittest.main()

