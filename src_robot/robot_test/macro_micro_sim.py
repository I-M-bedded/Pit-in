import argparse
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR in sys.path:
    sys.path.remove(SCRIPT_DIR)
sys.path.insert(0, os.path.join(SCRIPT_DIR, '..'))
from controller import ik
from robot_test.hardware import HardwareInterface
from robot_test.jacobian_solver import ReachMode
from robot_test.macro_micro import ControlConfig, SingleSideController
from robot_test.TCPcontroller import TCPcontroller


def _run_side_sim(ctrl, hw, side, target_xy, max_iter=200, noise_std=0.0002):
    """Run simulation for one side."""
    ctrl.set_target(target_xy, side=side, cartype=0)

    print(f"\n--- {side.upper()} PIN ---")
    print(f"  Target: {target_xy}, State: {ctrl.state.name}")

    def q_to_cpos(q):
        q_hw = q.copy()
        q_hw[2] = -q_hw[2]
        q_hw[3] = -q_hw[3]
        q_hw[4] = -q_hw[4]
        return [q_hw[i] * ctrl.topik.m2cnt[i] for i in range(7)]

    for i in range(max_iter):
        sim_c_pos = q_to_cpos(ctrl.q_cmd)
        noise = np.random.randn(2) * noise_std
        world_raw = np.array([target_xy[0] + noise[0], target_xy[1] + noise[1], 0.0])

        ctrl.step(world_raw=world_raw, current_c_pos=np.array(sim_c_pos))
        ctrl.q_cmd = hw.apply_safety_limits(ctrl.q_cmd)

        if ctrl.is_done:
            print(f"  [DONE] Converged in {i+1} iterations")
            break

        if (i + 1) % 10 == 0:
            pin = ctrl.kin.fk_pin(ctrl.q_cmd, side)
            err = np.linalg.norm(pin - target_xy)
            print(f"  iter {i+1}: state={ctrl.state.name}, pin={pin}, err={err:.6f}m")

    final_pin = ctrl.kin.fk_pin(ctrl.q_cmd, side)
    final_err = np.linalg.norm(final_pin - target_xy)
    print(f"  Final {side} pin: {final_pin}, Error: {final_err:.6f} m")
    return final_err


def simulate_pbvs():
    """Simulate the single-pin worker controller pipeline."""
    print("=" * 60)
    print("Macro-Micro PBVS Simulation (L -> R)")
    print("=" * 60)

    cfg = ControlConfig(version=4)
    topik = ik.Topik(cfg.version)
    hw = HardwareInterface(topik, cfg)
    ctrl = SingleSideController(topik, cfg)

    target_l = np.array([0.02, 0.70])
    _run_side_sim(ctrl, hw, 'left', target_l)

    target_r = np.array([0.02, -0.70])
    _run_side_sim(ctrl, hw, 'right', target_r)

    print("\n" + "=" * 60)
    print(f"  q_cmd(m,rad): {np.round(ctrl.q_cmd, 6)}")
    print(f"  q_cmd(cnt):   {hw.q_to_cnt(ctrl.q_cmd)}")
    print("=" * 60)


def test_coordinate_conversion():
    """Verify FK coordinate conversion and Jacobian consistency."""
    print("=" * 60)
    print("FK Right-Handed Coordinate Verification")
    print("=" * 60)

    topik = ik.Topik(4)
    topik.get_q([0, 0, 0, 0, 0, 0, 0])
    topik.fk()
    print(f"\nTest 1: q=[0]*7 (zero config)")
    print(f"  Left pin:  x={topik.x[0][0]:.6f}, y={topik.x[0][1]:.6f}")
    print(f"  Right pin: x={topik.x[1][0]:.6f}, y={topik.x[1][1]:.6f}")
    print(f"  Pin gap:   {topik.p2p:.6f}m")
    assert abs(topik.x[0][0]) > 0.1
    assert abs(topik.x[0][0] + topik.x[1][0]) < 0.001

    topik2 = ik.Topik(4)
    q_niro = topik2.set_xd(0, np.array([[0, 0, 0], [0, 0, 0]]))
    topik2.get_q(q_niro)
    topik2.fk()
    x_fk = topik2.x.copy()
    p2p_fk = topik2.p2p
    print(f"\nTest 2: Niro home position (IK->FK)")
    print(f"  Pin gap:   {p2p_fk:.6f}m (expected ~ {ik.niro_gap}m)")
    assert abs(p2p_fk - ik.niro_gap) < 0.02

    topik2.xd = x_fk.copy()
    topik2.num_ik()
    q_reik = topik2.qd
    max_diff = max(abs(q_niro[i] - q_reik[i]) for i in range(7))
    print(f"\nTest 3: FK->IK round trip")
    print(f"  Max diff:  {max_diff} counts")
    assert max_diff < 5

    topik3 = ik.Topik(4)
    q_test = topik3.set_xd(0, np.array([[0, 0, 0], [0, 0, 0]]))
    topik3.get_q(q_test)
    topik3.fk()
    topik3.get_J()
    j_analytical = topik3.J.copy()
    eps = 1e-6
    j_numerical = np.zeros((4, 7))
    q_base_m = topik3.qm.copy()
    for j in range(7):
        qm_plus = q_base_m.copy()
        qm_plus[j] += eps
        topik3.qm = qm_plus
        topik3.fk()
        x_plus = np.array([topik3.x[0][0], topik3.x[0][1], topik3.x[1][0], topik3.x[1][1]])
        qm_minus = q_base_m.copy()
        qm_minus[j] -= eps
        topik3.qm = qm_minus
        topik3.fk()
        x_minus = np.array([topik3.x[0][0], topik3.x[0][1], topik3.x[1][0], topik3.x[1][1]])
        j_numerical[:, j] = (x_plus - x_minus) / (2 * eps)
    j_err = np.max(np.abs(j_analytical - j_numerical))
    print(f"\nTest 4: Jacobian consistency")
    print(f"  Max element error: {j_err:.2e}")
    assert j_err < 1e-4

    print("\n" + "=" * 60)
    print("All coordinate conversion tests PASSED OK")
    print("=" * 60)


def simulate_unified():
    """Simulate the unified dual-pin PBVS pipeline."""
    print("=" * 60)
    print("Unified Dual-Pin PBVS Simulation")
    print("=" * 60)

    cfg = ControlConfig(version=4)
    cfg.micro_threshold = 0.0005
    cfg.max_stage_vel = 0.02
    cfg.diff_deadband = 0.001
    topik = ik.Topik(cfg.version)
    hw = HardwareInterface(topik, cfg)

    def q_to_cpos(q):
        q_hw = q.copy()
        q_hw[2] = -q_hw[2]
        q_hw[3] = -q_hw[3]
        q_hw[4] = -q_hw[4]
        return [q_hw[i] * topik.m2cnt[i] for i in range(7)]

    def run_case(case_name, target_l, target_r, drift_start=None, drift_delta=None):
        print(f"\n[CASE] {case_name}")
        print(f"  Target L: {target_l}")
        print(f"  Target R: {target_r}")

        topik.qm = np.zeros(7)
        topik.fk()
        ctrl = TCPcontroller(topik, cfg)
        mode, info = ctrl.set_targets(target_l, target_r, cartype=0)
        print(f"  Reachability: {mode.name}")
        for k, v in info.items():
            print(f"    {k}: {v}")

        if mode != ReachMode.BOTH:
            return

        noise_std = 0.0002
        for it in range(220):
            sim_cpos = q_to_cpos(ctrl.q_cmd)
            nl = np.random.randn(2) * noise_std
            nr = np.random.randn(2) * noise_std
            drift = np.zeros(2)
            if drift_start is not None and drift_delta is not None and it >= drift_start:
                drift = np.asarray(drift_delta, dtype=float)

            wl = np.array([target_l[0] + nl[0] - drift[0], target_l[1] + nl[1] - drift[1], 0.0])
            wr = np.array([target_r[0] + nr[0] + drift[0], target_r[1] + nr[1] + drift[1], 0.0])

            ctrl.step(world_raw_l=wl, world_raw_r=wr, current_c_pos=np.array(sim_cpos))
            ctrl.q_cmd = hw.apply_safety_limits(ctrl.q_cmd)

            if ctrl.is_done:
                print(f"  [{ctrl.finish_reason.name}] in {it+1} iterations")
                break

            if (it + 1) % 10 == 0:
                topik.qm = ctrl.q_cmd.copy()
                topik.fk()
                pl = np.array(topik.x[0][:2])
                pr = np.array(topik.x[1][:2])
                c_now, d_now = ctrl.kin.mode_decompose(pl, pr)
                c_target_live = ctrl.dual_ctrl.current_c_target
                d_target_live = ctrl.dual_ctrl.current_d_target
                print(f"  iter {it+1}: |e_c|={np.linalg.norm(c_now - c_target_live)*1000:.2f}mm  "
                      f"|e_d|={np.linalg.norm(d_now - d_target_live)*1000:.2f}mm")

        topik.qm = ctrl.q_cmd.copy()
        topik.fk()
        pl = np.array(topik.x[0][:2])
        pr = np.array(topik.x[1][:2])
        c_now, d_now = ctrl.kin.mode_decompose(pl, pr)
        c_target_live = ctrl.dual_ctrl.current_c_target
        d_target_live = ctrl.dual_ctrl.current_d_target
        c_target_nom, d_target_nom = ctrl.kin.mode_decompose(target_l, target_r)
        print(f"  Final live  |e_c|={np.linalg.norm(c_now - c_target_live)*1000:.4f}mm")
        print(f"  Final live  |e_d|={np.linalg.norm(d_now - d_target_live)*1000:.4f}mm")
        print(f"  Final nominal |e_c|={np.linalg.norm(c_now - c_target_nom)*1000:.4f}mm")
        print(f"  Final nominal |e_d|={np.linalg.norm(d_now - d_target_nom)*1000:.4f}mm")

    home_l = topik.xo_arr[0][0][:2].copy()
    home_r = topik.xo_arr[0][1][:2].copy()
    home_c = 0.5 * (home_l + home_r)
    home_d = home_r - home_l
    rot_deg = 2.0
    rot = np.array([
        [np.cos(np.deg2rad(rot_deg)), -np.sin(np.deg2rad(rot_deg))],
        [np.sin(np.deg2rad(rot_deg)), np.cos(np.deg2rad(rot_deg))],
    ])
    common_offset = np.array([0.005, 0.002])
    target_c = home_c + common_offset
    target_d = rot @ home_d
    target_l = target_c - 0.5 * target_d
    target_r = target_c + 0.5 * target_d

    print(f"  Home L: {home_l}, Home R: {home_r}")
    print(f"  Rotated differential by {rot_deg:.1f} deg for asymmetric target.")

    run_case("asymmetric_static", target_l, target_r)
    run_case(
        "asymmetric_with_diff_drift",
        target_l,
        target_r,
        drift_start=6,
        drift_delta=np.array([0.0020, -0.0012]),
    )

    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Macro-Micro PBVS Simulation")
    parser.add_argument('--test', action='store_true', help='Run coordinate tests')
    parser.add_argument('--sim', action='store_true', help='Run single-pin PBVS simulation')
    parser.add_argument('--unified', action='store_true', help='Run unified dual-pin simulation')
    parser.add_argument('--all', action='store_true', help='Run all tests')
    args = parser.parse_args()

    if args.test or args.all:
        test_coordinate_conversion()
        print()

    if args.sim or args.all:
        simulate_pbvs()

    if args.unified or args.all:
        simulate_unified()

    if not any([args.test, args.sim, args.unified, args.all]):
        test_coordinate_conversion()
        print()
        simulate_pbvs()
