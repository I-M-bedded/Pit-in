"""
Hand-Eye Calibration Validation Suite v2
matplotlib → plotly 로 교체 (NumPy 2.x 호환)
"""

import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import warnings
warnings.filterwarnings("ignore")

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    HAS_PLOTLY = True
except ImportError:
    HAS_PLOTLY = False
    print("plotly가 없습니다. 수치 결과만 출력합니다.")
    print("설치: pip3 install plotly kaleido")

# ─────────────────────────────────────────────
# 0. 설정
# ─────────────────────────────────────────────
OFFSET_CM  = np.array([5.7, 2.9, -21.51])
CSV_PATH   = "Pit-in/vision/Calibration_data.csv"
SWAP_AXES  = True
N_BOOTSTRAP = 500

# ─────────────────────────────────────────────
# 1. 데이터 로더
# ─────────────────────────────────────────────
def load_data(csv_path):
    df = pd.read_csv(csv_path)
    df = df[df["m1_id"] != "N/A"].reset_index(drop=True)
    rp = df[["robot_x", "robot_y"]].to_numpy()
    robot_pos_3d = np.hstack([rp, np.zeros((len(rp), 1))])
    robot_rot = np.array([
        R.from_euler("z", t, degrees=False).as_matrix()
        for t in df["robot_th"].to_numpy()
    ])
    m1 = df[["m1_x", "m1_y", "m1_z"]].to_numpy() * 100.0
    m2 = df[["m2_x", "m2_y", "m2_z"]].to_numpy() * 100.0
    return (robot_pos_3d, robot_rot,
            m1, df["m1_id"].to_numpy(),
            m2, df["m2_id"].to_numpy())

# ─────────────────────────────────────────────
# 2. 핵심 변환 / 오차 함수
# ─────────────────────────────────────────────
def world_xy(rotvec, rpos, rrot, mdata, mids, offset, swap):
    tR = R.from_rotvec(rotvec).as_matrix()
    valid = ~pd.isna(mids)
    if not np.any(valid):
        return None
    vp, vr, vm = rpos[valid], rrot[valid], mdata[valid].copy()
    if swap:
        vm[:, [0, 1]] = vm[:, [1, 0]]
    shifted = (tR @ vm.T).T - offset
    return (vp + np.einsum("nij,nj->ni", vr, shifted))[:, :2]

def error_fn(rotvec, rpos, rrot, m1, mi1, m2, mi2, offset, swap):
    res = []
    for md, mi in [(m1, mi1), (m2, mi2)]:
        xy = world_xy(rotvec, rpos, rrot, md, mi, offset, swap)
        if xy is None:
            continue
        c = np.mean(xy, axis=0)
        res.append((xy - c).flatten())
    return np.concatenate(res)

def solve(rpos, rrot, m1, mi1, m2, mi2, offset, swap):
    res = least_squares(error_fn, np.zeros(3),
                        args=(rpos, rrot, m1, mi1, m2, mi2, offset, swap),
                        method="trf")
    return res.x, res.cost

def per_sample_err(rotvec, rpos, rrot, m1, mi1, m2, mi2, offset, swap):
    errs = {}
    for md, mi in [(m1, mi1), (m2, mi2)]:
        valid_idx = np.where(~pd.isna(mi))[0]
        if len(valid_idx) == 0:
            continue
        xy = world_xy(rotvec, rpos, rrot, md, mi, offset, swap)
        c  = np.mean(xy, axis=0)
        for k, idx in enumerate(valid_idx):
            e = np.linalg.norm(xy[k] - c)
            errs[idx] = max(errs.get(idx, 0), e)
    return errs

# ─────────────────────────────────────────────
# 3. 전체 솔루션
# ─────────────────────────────────────────────
print("데이터 로드 중...")
rpos, rrot, m1, mi1, m2, mi2 = load_data(CSV_PATH)
N = len(rpos)
print(f"  유효 샘플 수: {N}")

best_rv, _ = solve(rpos, rrot, m1, mi1, m2, mi2, OFFSET_CM, SWAP_AXES)
best_euler = R.from_rotvec(best_rv).as_euler("xyz", degrees=True)
print(f"  최적 Euler (XYZ°): {best_euler.round(3)}")

full_errs = per_sample_err(best_rv, rpos, rrot, m1, mi1, m2, mi2, OFFSET_CM, SWAP_AXES)
err_vals  = np.array(list(full_errs.values()))
mean_err  = err_vals.mean()
print(f"  전체 평균 XY 오차: {mean_err:.4f} cm  |  std: {err_vals.std():.4f}  |  max: {err_vals.max():.4f}")

# ─────────────────────────────────────────────
# 4. Leave-One-Out CV
# ─────────────────────────────────────────────
print("\nLOO Cross-Validation...")
loo_errs = []
for i in range(N):
    mask = np.ones(N, dtype=bool); mask[i] = False
    try:
        rv_l, _ = solve(rpos[mask], rrot[mask],
                        m1[mask], mi1[mask],
                        m2[mask], mi2[mask],
                        OFFSET_CM, SWAP_AXES)
        fn = error_fn(rv_l, rpos[mask], rrot[mask],
                      m1[mask], mi1[mask], m2[mask], mi2[mask],
                      OFFSET_CM, SWAP_AXES)
        loo_errs.append(np.mean(np.linalg.norm(fn.reshape(-1, 2), axis=1)))
    except Exception:
        pass

loo_errs = np.array(loo_errs)
loo_mean = loo_errs.mean()
loo_gap  = abs(loo_mean - mean_err)
print(f"  LOO 평균: {loo_mean:.4f} cm  |  gap: {loo_gap:.4f} cm  → "
      f"{'과적합 낮음 ✅' if loo_gap < 0.3 else '주의 필요 ⚠️'}")

# ─────────────────────────────────────────────
# 5. Bootstrap CI
# ─────────────────────────────────────────────
print(f"\nBootstrap CI (n={N_BOOTSTRAP})...")
np.random.seed(42)
boot_eulers = []
for _ in range(N_BOOTSTRAP):
    idx = np.random.choice(N, N, replace=True)
    try:
        rv_b, _ = solve(rpos[idx], rrot[idx],
                        m1[idx], mi1[idx], m2[idx], mi2[idx],
                        OFFSET_CM, SWAP_AXES)
        boot_eulers.append(R.from_rotvec(rv_b).as_euler("xyz", degrees=True))
    except Exception:
        pass
boot_eulers = np.array(boot_eulers)

for i, name in enumerate(["Rx", "Ry", "Rz"]):
    lo, hi = np.percentile(boot_eulers[:, i], [2.5, 97.5])
    width = hi - lo
    quality = "안정 ✅" if width < 10 else ("보통 ⚠️" if width < 20 else "불안정 ❌")
    print(f"  {name}: {best_euler[i]:.2f}° → 95%CI [{lo:.2f}, {hi:.2f}] "
          f"(±{width/2:.1f}°) {quality}")

# ─────────────────────────────────────────────
# 6. Offset 민감도
# ─────────────────────────────────────────────
print("\nOffset 민감도...")
perturb = np.linspace(-2.0, 2.0, 21)
sensitivity = {}
for ai, an in enumerate(["x", "y", "z"]):
    errs = []
    for d in perturb:
        off = OFFSET_CM.copy(); off[ai] += d
        try:
            rv_s, _ = solve(rpos, rrot, m1, mi1, m2, mi2, off, SWAP_AXES)
            fn = error_fn(rv_s, rpos, rrot, m1, mi1, m2, mi2, off, SWAP_AXES)
            errs.append(np.mean(np.linalg.norm(fn.reshape(-1, 2), axis=1)))
        except Exception:
            errs.append(np.nan)
    sensitivity[an] = np.array(errs)
    g = np.gradient(errs, perturb)
    print(f"  offset_{an} 평균 민감도: {np.nanmean(np.abs(g)):.4f} cm/cm")

# ─────────────────────────────────────────────
# 7. World XY scatter
# ─────────────────────────────────────────────
xy_before = world_xy(np.zeros(3), rpos, rrot, m1, mi1, OFFSET_CM, SWAP_AXES)
xy_m1 = world_xy(best_rv, rpos, rrot, m1, mi1, OFFSET_CM, SWAP_AXES)
xy_m2 = world_xy(best_rv, rpos, rrot, m2, mi2, OFFSET_CM, SWAP_AXES)

# ─────────────────────────────────────────────
# 8. CSV 요약
# ─────────────────────────────────────────────
import os; os.makedirs("output", exist_ok=True)
summary_rows = [
    ("전체 평균 XY 오차 (cm)",   round(mean_err, 4)),
    ("전체 최대 오차 (cm)",      round(err_vals.max(), 4)),
    ("전체 오차 std (cm)",       round(err_vals.std(), 4)),
    ("LOO 평균 오차 (cm)",       round(loo_mean, 4)),
    ("LOO gap (cm)",             round(loo_gap, 4)),
    ("Euler Rx (deg)",           round(best_euler[0], 4)),
    ("Euler Ry (deg)",           round(best_euler[1], 4)),
    ("Euler Rz (deg)",           round(best_euler[2], 4)),
    ("Bootstrap CI Rx lo",       round(np.percentile(boot_eulers[:,0], 2.5), 3)),
    ("Bootstrap CI Rx hi",       round(np.percentile(boot_eulers[:,0], 97.5), 3)),
    ("Bootstrap CI Ry lo",       round(np.percentile(boot_eulers[:,1], 2.5), 3)),
    ("Bootstrap CI Ry hi",       round(np.percentile(boot_eulers[:,1], 97.5), 3)),
    ("Bootstrap CI Rz lo",       round(np.percentile(boot_eulers[:,2], 2.5), 3)),
    ("Bootstrap CI Rz hi",       round(np.percentile(boot_eulers[:,2], 97.5), 3)),
]
pd.DataFrame(summary_rows, columns=["metric", "value"]).to_csv(
    "output/handeye_validation_summary.csv", index=False
)

# ─────────────────────────────────────────────
# 9. Plotly 시각화
# ─────────────────────────────────────────────
if not HAS_PLOTLY:
    print("\n✅ 수치 분석 완료 (plotly 없음 → 그래프 생략)")
    print("✅ 저장: output/handeye_validation_summary.csv")
    raise SystemExit

BG   = "#0f0f0f"
S_BG = "#161616"
ACCENT = "#4f98a3"
WARN   = "#fdab43"
ERR    = "#dd6974"
TXT    = "#e0e0e0"
GRID   = "#2a2a2a"

fig = make_subplots(
    rows=3, cols=3,
    subplot_titles=[
        "World XY — 보정 전",    "World XY — 보정 후",    "샘플별 오차 분포",
        "LOO 교차 검증 오차",    "Bootstrap CI — Rx",     "Bootstrap CI — Ry",
        "Offset 민감도 — X",     "Offset 민감도 — Y",     "Offset 민감도 — Z (높이)",
    ],
    vertical_spacing=0.12,
    horizontal_spacing=0.08,
)

# ── (1,1) Before ──
if xy_before is not None:
    c0 = xy_before.mean(0)
    sp0 = np.mean(np.linalg.norm(xy_before - c0, axis=1))
    fig.add_trace(go.Scatter(x=xy_before[:,0], y=xy_before[:,1],
        mode="markers", marker=dict(color=ERR, size=6, opacity=0.75),
        name="before", showlegend=False), row=1, col=1)
    fig.add_trace(go.Scatter(x=[c0[0]], y=[c0[1]], mode="markers",
        marker=dict(symbol="cross", size=14, color="white"),
        showlegend=False), row=1, col=1)
    fig.add_annotation(xref="x1 domain", yref="y1 domain",
        x=0.05, y=0.95, text=f"spread {sp0:.2f} cm",
        font=dict(color=ERR, size=10), showarrow=False, row=1, col=1)

# ── (1,2) After ──
for xy, color, name in [(xy_m1, ACCENT, "m1"), (xy_m2, WARN, "m2")]:
    if xy is None: continue
    ctr = xy.mean(0)
    fig.add_trace(go.Scatter(x=xy[:,0], y=xy[:,1], mode="markers",
        marker=dict(color=color, size=6, opacity=0.8),
        name=name, showlegend=True, legendgroup="after"), row=1, col=2)
    fig.add_trace(go.Scatter(x=[ctr[0]], y=[ctr[1]], mode="markers",
        marker=dict(symbol="cross", size=14, color="white"),
        showlegend=False), row=1, col=2)

# ── (1,3) Histogram ──
p95 = np.percentile(err_vals, 95)
fig.add_trace(go.Histogram(x=err_vals, nbinsx=12,
    marker_color=ACCENT, opacity=0.85, showlegend=False), row=1, col=3)
for val, color, label in [(mean_err, WARN, f"mean={mean_err:.2f}"),
                           (p95,      ERR,  f"P95={p95:.2f}")]:
    fig.add_vline(x=val, line_color=color, line_dash="dash",
                  annotation_text=label, annotation_font_color=color,
                  annotation_font_size=9, row=1, col=3)

# ── (2,1) LOO bars ──
loo_colors = [ERR if e > 1.5*mean_err else ACCENT for e in loo_errs]
fig.add_trace(go.Bar(x=list(range(len(loo_errs))), y=loo_errs,
    marker_color=loo_colors, showlegend=False), row=2, col=1)
fig.add_hline(y=mean_err, line_color=WARN, line_dash="dash",
              annotation_text=f"train {mean_err:.2f}", row=2, col=1)
fig.add_hline(y=loo_mean, line_color="white", line_dash="dot",
              annotation_text=f"LOO {loo_mean:.2f}", row=2, col=1)

# ── (2,2)(2,3) Bootstrap Rx, Ry ──
for col_i, (idx, color) in enumerate([(0, ACCENT), (1, WARN)], start=2):
    vals = boot_eulers[:, idx]
    lo, hi = np.percentile(vals, [2.5, 97.5])
    fig.add_trace(go.Histogram(x=vals, nbinsx=25,
        marker_color=color, opacity=0.8, showlegend=False), row=2, col=col_i)
    fig.add_vline(x=best_euler[idx], line_color="white", line_dash="solid",
                  row=2, col=col_i)
    for v in [lo, hi]:
        fig.add_vline(x=v, line_color=ERR, line_dash="dot",
                      row=2, col=col_i)

# ── (3,1~3) Sensitivity ──
s_colors = [ACCENT, WARN, ERR]
for col_i, (an, color) in enumerate(zip(["x","y","z"], s_colors), start=1):
    e = sensitivity[an]
    fig.add_trace(go.Scatter(x=perturb, y=e,
        line=dict(color=color, width=2), mode="lines",
        showlegend=False), row=3, col=col_i)
    fig.add_vline(x=0, line_color="white", line_dash="dash", row=3, col=col_i)
    fig.add_hline(y=mean_err, line_color=WARN, line_dash="dot", row=3, col=col_i)

# ── 레이아웃 ──
axis_style = dict(
    showgrid=True, gridcolor=GRID, zeroline=False,
    color=TXT, linecolor="#333"
)
fig.update_layout(
    height=900, width=1350,
    paper_bgcolor=BG, plot_bgcolor=S_BG,
    font=dict(color=TXT, size=10),
    title=dict(text="Hand-Eye Calibration Validation Report",
               font=dict(size=16, color=TXT), x=0.5),
    legend=dict(bgcolor="#1c1c1c", bordercolor="#333", font=dict(color=TXT)),
    margin=dict(t=80, b=40, l=50, r=30),
)
fig.update_xaxes(**axis_style)
fig.update_yaxes(**axis_style)
for ann in fig.layout.annotations:
    ann.font.color = TXT
    ann.font.size  = 11

fig.write_image("output/handeye_validation.png", scale=1.5)

print("\n✅ 저장 완료: output/handeye_validation.png")
print("✅ 저장 완료: output/handeye_validation_summary.csv")