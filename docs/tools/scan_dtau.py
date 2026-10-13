# -*- coding: utf-8 -*-
"""扫描磁样本的时间对齐量 Δτ ∈ [0, 1 ms]（地磁 1 kHz）。

对每个磁样本 k（落在帧 j=idx[k]），其真实采样时刻比帧 j 早 Δτ，故
    q_true = q[j] ⊗ exp(−ω[j]·Δτ)          （体坐标右乘，用陀螺反旋回去）
    b_nav  = R(q_true) · b_body_unit(k)
若 Δτ 正确，b_nav 应最稳 ⟹ 取"b_nav 围绕其均值的 RMS 夹角"为代价函数，扫出谷底。
"""
import sys
sys.path.insert(0, '.')
import numpy as np
import magmap

SRC = r'R:\rec_20261013_082322.bin'
fr = magmap.Frames(SRC)
cal, cinfo = magmap.calibrate(fr, prefer='SET')
FS = 1.0 / magmap.NOMINAL_DT

m = fr.new_mask(); idx = np.where(m)[0]
raw = fr.mag[m]; polm = fr.pol[m]
gy = fr.gyro_dps.astype(np.float64)                     # dps, 体坐标
q = fr.quat.astype(np.float64); q /= np.linalg.norm(q, axis=1, keepdims=True)

bb = np.full((len(idx), 3), np.nan); ok = np.zeros(len(idx), bool)
for k in range(len(idx)):
    try:
        v, _ = magmap.mag_vector(raw[k], polm[k], cal, strict_phase=True)
        n = np.linalg.norm(v)
        if n > 1e-9: bb[k] = v / n; ok[k] = True
    except Exception:
        pass

def quat_R(qq):
    w, x, y, z = qq[..., 0], qq[..., 1], qq[..., 2], qq[..., 3]
    return np.stack([
        np.stack([1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], -1),
        np.stack([2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], -1),
        np.stack([2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)], -1)], -2)

def qmul(a, b):
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([aw*bw-ax*bx-ay*by-az*bz,
                     aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx,
                     aw*bz+ax*by-ay*bx+az*bw], -1)

def expmap(v):                                          # 旋转向量 -> 四元数
    th = np.linalg.norm(v, axis=-1, keepdims=True)
    h = 0.5 * th
    s = np.where(th > 1e-12, np.sin(h) / np.maximum(th, 1e-12), 0.5)
    return np.concatenate([np.cos(h), v * s], -1)

J = idx[ok]; B = bb[ok]
spd = np.linalg.norm(gy[J], axis=1)
mov = spd > 500.0                                       # 只看运动样本（位移效应最大）

def cost(tau_ms):
    tau = tau_ms * 1e-3
    qc = qmul(q[J], expmap(-np.radians(gy[J]) * tau))
    bn = np.einsum('nij,nj->ni', quat_R(qc), B)
    bn /= np.maximum(np.linalg.norm(bn, axis=1, keepdims=True), 1e-12)
    def rms(sel):
        r = bn[sel].mean(0); r /= np.linalg.norm(r)
        return np.degrees(np.arccos(np.clip(bn[sel] @ r, -1, 1))).std()
    return rms(np.ones(len(bn), bool)), rms(mov)

print('Δτ 扫描（Δτ ∈ [0, 1 ms]，地磁 1 kHz）；代价 = b_nav 围绕均值的 RMS 夹角')
print('%-9s %14s %14s' % ('Δτ (ms)', '全体 RMS°', '运动样本 RMS°'))
best = None
for t in np.arange(0.0, 1.0001, 0.05):
    a, b = cost(t)
    mark = ''
    if best is None or b < best[1]: best = (t, b); mark = ''
    print('%-9.2f %14.4f %14.4f' % (t, a, b))
print()
print('★ 运动样本最优 Δτ = %.2f ms  (RMS %.4f°)' % best)
# 精细扫描
lo, hi = max(0.0, best[0]-0.1), min(1.0, best[0]+0.1)
print()
print('精细扫描 %.2f~%.2f ms:' % (lo, hi))
bb2 = None
for t in np.arange(lo, hi+1e-9, 0.01):
    a, b = cost(t)
    print('   Δτ=%.3f ms   运动 RMS %.4f°' % (t, b))
    if bb2 is None or b < bb2[1]: bb2 = (t, b)
print()
print('★★ 最优 Δτ = %.3f ms  ⟹ 运动 RMS 从 Δτ=0 的 %.4f° 降到 %.4f°'
      % (bb2[0], cost(0.0)[1], bb2[1]))
print('   （若谷底明显 ⟹ 18° 是纯时间错配，磁与陀螺都对；若无谷 ⟹ 是 ESKF 姿态的真实每转误差）')
