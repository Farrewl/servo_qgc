#!/usr/bin/env python3
"""tests/test_mavlink_util.py — Unit test konversi MANUAL_CONTROL.

Jalankan dari folder ini:
    python3 -m unittest discover -s tests -v
"""

import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mavlink_util import (R_FULL, Z_MAX, Z_MIN, Z_NEUTRAL,
                          clamp1, manual_to_vw, vw_to_manual)


def _msg(z, r):
    """Tiruan pesan MAVLink MANUAL_CONTROL (yang dipakai manual_to_vw)."""
    return SimpleNamespace(z=z, r=r)


class TestClamp(unittest.TestCase):
    def test_rentang(self):
        self.assertEqual(clamp1(-5), -1.0)
        self.assertEqual(clamp1(5), 1.0)
        self.assertEqual(clamp1(0.3), 0.3)


class TestManualToVw(unittest.TestCase):
    def test_netral_zero(self):
        self.assertEqual(manual_to_vw(_msg(Z_NEUTRAL, 0)), (0.0, 0.0))

    def test_maju_mundur_penuh(self):
        self.assertEqual(manual_to_vw(_msg(Z_MAX, 0))[0], 1.0)
        self.assertEqual(manual_to_vw(_msg(Z_MIN, 0))[0], -1.0)

    def test_yaw_penuh(self):
        self.assertEqual(manual_to_vw(_msg(Z_NEUTRAL, R_FULL))[1], 1.0)
        self.assertEqual(manual_to_vw(_msg(Z_NEUTRAL, -R_FULL))[1], -1.0)

    def test_z_liar_dianggap_netral(self):
        # Nilai di luar protokol -> netral, bukan gas penuh (pengaman).
        self.assertEqual(manual_to_vw(_msg(9999, 0)), (0.0, 0.0))

    def test_objek_rusak_netral(self):
        self.assertEqual(manual_to_vw(_msg("bukan-angka", None)), (0.0, 0.0))


class TestVwToManual(unittest.TestCase):
    def test_mapping_penuh(self):
        self.assertEqual(vw_to_manual(1.0, 1.0), (int(Z_MAX), int(R_FULL)))
        self.assertEqual(vw_to_manual(-1.0, -1.0), (int(Z_MIN), -int(R_FULL)))

    def test_netral(self):
        self.assertEqual(vw_to_manual(0.0, 0.0), (int(Z_NEUTRAL), 0))

    def test_setengah(self):
        z, r = vw_to_manual(0.5, -0.5)
        self.assertEqual(z, 750)
        self.assertEqual(r, -64)   # round(0.5*127)=64

    def test_clamp_sebelum_kirim(self):
        self.assertEqual(vw_to_manual(2.0, -2.0),
                         (int(Z_MAX), -int(R_FULL)))


class TestRoundTrip(unittest.TestCase):
    def test_kirim_balik_ke_kirim(self):
        # (v,w) -> nilai mentah -> (v,w), toleransi 1 langkah kuantisasi.
        for v, w in [(0.5, 0.5), (-1.0, 0.25), (0.0, -0.75), (0.25, -0.25)]:
            z, r = vw_to_manual(v, w)
            v2, w2 = manual_to_vw(_msg(z, r))
            self.assertAlmostEqual(v, v2, delta=0.01)
            self.assertAlmostEqual(w, w2, delta=0.01)


if __name__ == "__main__":
    unittest.main()