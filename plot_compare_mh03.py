"""Plot MH_03: VINS loop/noloop aligned to GT via SE(3) similarity transform.

This is the standard VINS evaluation approach:
  - GT is the reference (Vicon world frame)
  - Each VINS trajectory is aligned to GT via Umeyama (s*R, t)
    so they all live in the same world frame.
  - Time is aligned by mapping each VINS trajectory's first frame
    onto GT's first frame (by closest timestamp), then re-basing
    everything to GT's t0.
  - Result: shape, scale, and direction match GT; the residual
    is purely "local drift" between keyframes.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_vins_csv(path):
    """Load a vins_result_*.csv: t (sec or ns), x, y, z, qw, qx, qy, qz, ..."""
    rows = []
    with open(path) as f:
        for line in f:
            p = line.strip().split(',')
            if len(p) >= 4 and not line.startswith('#'):
                try: rows.append([float(x) for x in p[:4]])
                except: pass
    a = np.array(rows)
    if a[0, 0] > 1e15: a[:, 0] /= 1e9
    # Dedup: keep last write per timestamp
    _, inv = np.unique(a[:, 0], return_inverse=True)
    out = np.zeros((inv.max() + 1, 4))
    last = {}
    for i, k in enumerate(inv):
        last[k] = i
    for k, i in last.items():
        out[k] = a[i]
    return out


def resample_to_ref(est_t, est_xyz, ref_t):
    idx = np.searchsorted(est_t, ref_t)
    idx = np.clip(idx, 1, len(est_t) - 1)
    left = idx - 1
    choose_left = (ref_t - est_t[left]) <= (est_t[idx] - ref_t)
    nearest = np.where(choose_left, left, idx)
    return est_xyz[nearest]


def umeyama(src, dst):
    """Similarity transform: dst = s * R @ src + t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    src_c, dst_c = src - mu_s, dst - mu_d
    H = src_c.T @ dst_c / len(src)
    U, D, Vt = np.linalg.svd(H)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = Vt.T @ S @ U.T
    var = (src_c ** 2).sum() / len(src)
    s = (D * np.diag(S)).sum() / var
    t = mu_d - s * R @ mu_s
    return s, R, t


def align_xyz(xyz, s, R, t):
    return (s * (R @ xyz.T)).T + t


# ===== Load =====
gt = np.loadtxt('experiments/sparse_compare/results/mh03_gt.csv', delimiter=' ', skiprows=1)
gt_t = gt[:, 0] - gt[0, 0]
gt_xyz = gt[:, 1:4]

loop_raw = load_vins_csv('output/vins_result_loop.csv')   # sparse ON + fixed loop
noloop_raw = load_vins_csv('output/vins_result_no_loop.csv')  # sparse ON, no loop

loop_t = loop_raw[:, 0].copy()
loop_xyz = loop_raw[:, 1:4].copy()
noloop_t = noloop_raw[:, 0].copy()
noloop_xyz = noloop_raw[:, 1:4].copy()

# Common time window: shift VINS times so their first sample aligns
# with GT's first sample (bag-start sync), then crop to GT range.
loop_t = loop_t - loop_t[0]
noloop_t = noloop_t - noloop_t[0]

t_lo = 0.0
t_hi = min(gt_t[-1], loop_t[-1], noloop_t[-1])
gt_t_c = gt_t[(gt_t >= t_lo) & (gt_t <= t_hi)]
gt_c = gt_xyz[(gt_t >= t_lo) & (gt_t <= t_hi)]

def prep(t, xyz):
    m = (t >= t_lo) & (t <= t_hi)
    return t[m], xyz[m]

loop_t, loop_xyz = prep(loop_t, loop_xyz)
noloop_t, noloop_xyz = prep(noloop_t, noloop_xyz)
print(f'after rebasing: loop t=[{loop_t[0]:.2f},{loop_t[-1]:.2f}]s, noloop t=[{noloop_t[0]:.2f},{noloop_t[-1]:.2f}]s')
print(f'GT t=[{gt_t_c[0]:.2f},{gt_t_c[-1]:.2f}]s')

# Resample to GT timestamps
loop_xyz_g = resample_to_ref(loop_t, loop_xyz, gt_t_c)
noloop_xyz_g = resample_to_ref(noloop_t, noloop_xyz, gt_t_c)

# Align via Umeyama
s_l, R_l, t_l = umeyama(loop_xyz_g, gt_c)
s_n, R_n, t_n = umeyama(noloop_xyz_g, gt_c)

loop_a = align_xyz(loop_xyz, s_l, R_l, t_l)
noloop_a = align_xyz(noloop_xyz, s_n, R_n, t_n)

# back-to-origin in aligned frame
d_gt = np.linalg.norm(gt_c[-1] - gt_c[0])
d_loop = np.linalg.norm(loop_a[-1] - loop_a[0])
d_noloop = np.linalg.norm(noloop_a[-1] - noloop_a[0])

# RMSE after alignment
err_loop = np.sqrt(((loop_a[resample_to_ref(loop_t, np.arange(len(loop_a)), gt_t_c)] - gt_c) ** 2).sum(axis=1).mean())
err_noloop = np.sqrt(((noloop_a[resample_to_ref(noloop_t, np.arange(len(noloop_a)), gt_t_c)] - gt_c) ** 2).sum(axis=1).mean())

print('===== Summary =====')
print(f'back-to-origin (aligned frame):')
print(f'  GT           {d_gt:.4f}m')
print(f'  loop         {d_loop:.4f}m   scale={s_l:.4f}  RMSE={err_loop:.3f}m')
print(f'  noloop       {d_noloop:.4f}m   scale={s_n:.4f}  RMSE={err_noloop:.3f}m')

# ===== x-t, y-t, z-t =====
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
for ax, idx, name in zip(axes, [0, 1, 2], ['x', 'y', 'z']):
    ax.plot(gt_t_c, gt_c[:, idx], 'k-', lw=2.0,
            label=f'Ground truth (Leica)  back={d_gt:.3f}m')
    ax.plot(loop_t, loop_a[:, idx], color='#1f77b4', lw=1.2,
            label=f'VINS (sparse ON + fixed loop)  back={d_loop:.3f}m  scale={s_l:.3f}')
    ax.plot(noloop_t, noloop_a[:, idx], color='#d62728', lw=1.0, alpha=0.85,
            label=f'VINS (sparse ON, no loop)  back={d_noloop:.3f}m  scale={s_n:.3f}')
    ax.set_ylabel(f'{name} [m]')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8)
axes[-1].set_xlabel('t [s]')
axes[0].set_title(
    'MH_03: VINS vs Ground Truth  (SE(3)-aligned to GT)\n'
    f'back-to-origin: GT={d_gt:.3f}m  loop={d_loop:.3f}m  noloop={d_noloop:.3f}m',
    fontsize=11)
fig.tight_layout()
fig.savefig('output/fixed_aligned/fig_xt_yt_zt.png', dpi=150, bbox_inches='tight')
plt.close(fig)
print('saved fig_xt_yt_zt.png')

# ===== 2D top-down =====
fig, ax = plt.subplots(figsize=(11, 9))
ax.plot(gt_c[:, 0], gt_c[:, 1], 'k-', lw=2.0,
        label=f'Ground truth (Leica)  back={d_gt:.3f}m')
ax.plot(loop_a[:, 0], loop_a[:, 1], color='#1f77b4', lw=1.4,
        label=f'VINS (sparse ON + fixed loop)  back={d_loop:.3f}m')
ax.plot(noloop_a[:, 0], noloop_a[:, 1], color='#d62728', lw=1.0, alpha=0.7,
        label=f'VINS (sparse ON, no loop)  back={d_noloop:.3f}m')

# Start (GT start)
ax.scatter(gt_c[0, 0], gt_c[0, 1], c='gray', s=80, marker='o',
           zorder=10, label='start (GT[0])')

# End points
ax.scatter(loop_a[-1, 0], loop_a[-1, 1],   c='#1f77b4', s=220, marker='*', zorder=10)
ax.scatter(noloop_a[-1, 0], noloop_a[-1, 1], c='#d62728', s=220, marker='*', zorder=10)
ax.scatter(gt_c[-1, 0], gt_c[-1, 1],         c='k', s=220, marker='*', zorder=10)

ax.annotate(f'loop end\nback={d_loop:.3f}m',
            xy=(loop_a[-1, 0], loop_a[-1, 1]),
            xytext=(20, 20), textcoords='offset points', fontsize=10,
            color='#1f77b4',
            arrowprops=dict(arrowstyle='->', color='#1f77b4'))
ax.annotate(f'noloop end\nback={d_noloop:.3f}m',
            xy=(noloop_a[-1, 0], noloop_a[-1, 1]),
            xytext=(-100, 10), textcoords='offset points', fontsize=10,
            color='#d62728',
            arrowprops=dict(arrowstyle='->', color='#d62728'))

ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('MH_03: 2D top-down trajectory  (SE(3)-aligned to GT)\n'
             f'back-to-origin: GT={d_gt:.3f}m  loop={d_loop:.3f}m  noloop={d_noloop:.3f}m')
ax.grid(True, alpha=0.3)
ax.legend(loc='best', fontsize=9)
ax.axis('equal')
fig.tight_layout()
fig.savefig('output/fixed_aligned/fig_xy_topdown.png', dpi=150, bbox_inches='tight')
plt.close(fig)
print('saved fig_xy_topdown.png')

# ===== 3D =====
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(gt_c[:, 0], gt_c[:, 1], gt_c[:, 2], 'k-', lw=2.0,
        label=f'Ground truth (Leica)  back={d_gt:.3f}m')
ax.plot(loop_a[:, 0], loop_a[:, 1], loop_a[:, 2], color='#1f77b4', lw=1.2,
        label=f'VINS (sparse ON + fixed loop)  back={d_loop:.3f}m')
ax.plot(noloop_a[:, 0], noloop_a[:, 1], noloop_a[:, 2], color='#d62728', lw=1.0, alpha=0.7,
        label=f'VINS (sparse ON, no loop)  back={d_noloop:.3f}m')
ax.scatter(loop_a[-1, 0], loop_a[-1, 1], loop_a[-1, 2], c='#1f77b4', s=180, marker='*', zorder=10)
ax.scatter(noloop_a[-1, 0], noloop_a[-1, 1], noloop_a[-1, 2], c='#d62728', s=180, marker='*', zorder=10)
ax.scatter(gt_c[-1, 0], gt_c[-1, 1], gt_c[-1, 2], c='k', s=180, marker='*', zorder=10)
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')
ax.set_title('MH_03: 3D trajectory  (SE(3)-aligned to GT)')
ax.legend(loc='best', fontsize=9)
fig.tight_layout()
fig.savefig('output/fixed_aligned/fig_xyz.png', dpi=150, bbox_inches='tight')
plt.close(fig)
print('saved fig_xyz.png')