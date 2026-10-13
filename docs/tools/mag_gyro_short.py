# -*- coding: utf-8 -*-
"""短基线互证：相邻两个磁样本之间（Δt≈1.3 ms），磁自身转过的角 vs 陀螺积分的角。

若两者一致  ⟹ 磁与陀螺在【瞬时尺度】上互相印证 ⟹ 那个 18° 的每转一次误差
                既不在磁、也不在陀螺，而来自 ESKF 姿态（滤波滞后/其他）。
若两者不一致 ⟹ 瞬时尺度上就能分出是谁错，且不掺任何累计漂移。
"""
import sys
sys.path.insert(0, '.')
import numpy as np
import magmap

fr = magmap.Frames(r'R:\rec_20261013_082322.bin')
cal, cinfo = magmap.calibrate(fr, prefer='SET')
FS = 1.0 / magmap.NOMINAL_DT

m = fr.new_mask(); idx = np.where(m)[0]
raw = fr.mag[m]; polm = fr.pol[m]
g = np.linalg.norm(fr.gyro_dps.astype(np.float64), axis=1)
vec = np.full((len(idx), 3), np.nan); ok = np.zeros(len(idx), bool)
for k in range(len(idx)):
    try:
        v, _ = magmap.mag_vector(raw[k], polm[k], cal, strict_phase=True)
        n = np.linalg.norm(v)
        if n > 1e-9: vec[k] = v / n; ok[k] = True
    except Exception:
        pass

# 相邻可用样本对
p, q2 = [], []
for k in range(1, len(idx)):
    if ok[k] and ok[k-1]:
        dt = (idx[k] - idx[k-1]) / FS
        if dt < 0.01:                       # 只取相邻的（<10ms，排除跨缺口）
            p.append(k-1); q2.append(k)
p = np.array(p); q2 = np.array(q2)
dt = (idx[q2] - idx[p]) / FS
d_mag = np.degrees(np.arccos(np.clip((vec[p] * vec[q2]).sum(1), -1, 1)))
# 陀螺在 [idx[p], idx[q2]] 上的积分
d_gyro = np.array([np.sum(g[idx[p][i]:idx[q2][i]+1]) / FS for i in range(len(p))])
ratio = d_mag / np.maximum(d_gyro, 1e-9)
spd = np.array([np.mean(g[idx[p][i]:idx[q2][i]+1]) for i in range(len(p))])
diffs = d_mag - d_gyro

print('相邻磁样本对 %d 对   Δt 中位 %.3f ms' % (len(p), np.median(dt) * 1e3))
print('每对转角:  磁 中位 %.4f°   陀螺 中位 %.4f°   比值 中位 %.5f' %
      (np.median(d_mag), np.median(d_gyro), np.median(ratio)))
print('两者之差 (磁−陀螺):  中位 %+.4f°   σ %.4f°   95%%|·| %.4f°   max|·| %.4f°' %
      (np.median(diffs), np.std(diffs), np.percentile(np.abs(diffs), 95), np.abs(diffs).max()))
print()
print('%-10s %8s %12s %12s %12s %14s' % ('转速段(dps)', '对数', '磁转角°', '陀螺转角°', '比值', '差的中位°'))
for lo, hi in ((0, 100), (100, 500), (500, 1000), (1000, 1500), (1500, 2500)):
    s = (spd >= lo) & (spd < hi)
    if s.sum() < 50: continue
    print('%-10s %8d %12.4f %12.4f %12.5f %+14.4f' %
          ('%d~%d' % (lo, hi), s.sum(), np.median(d_mag[s]), np.median(d_gyro[s]),
           np.median(ratio[s]), np.median(diffs[s])))
print()
print('★ 若"差的中位"随转速单调增大而"比值"稳定 ⟹ 比值就是陀螺标度误差, 磁是干净的')
print('★ 若"差的中位"≈0 而"比值"≈1  ⟹ 磁与陀螺在瞬时尺度完全互证, 18° 来自 ESKF')
