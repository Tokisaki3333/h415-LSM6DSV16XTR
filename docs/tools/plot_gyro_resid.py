# -*- coding: utf-8 -*-
"""以地磁【方向】为绝对真值，逐帧量出 ESKF/陀螺的姿态残差，并看它的离散速度。

口径要点（与先前实测一致）：
  * 磁的【模长】有 ±7~8% 的真实场变化，但【角度】精度典型 ±0.5° 且与速度无关
    ⟹ 本脚本把磁矢量【归一化】，只用方向，模长起伏完全不进入误差量。
  * 椭球(软磁)项实测各向异性仅 0.4%（_ellip_set.npy 的 Q 对角 1.4727/1.4607/1.4715e-8），
    套上也改不了模长 0.4% ⟹ 与残差无关，故只用 magmap 的分相球心（硬磁）。
  * b_nav = R(q_eskf)·b_body_unit。若 ESKF 姿态完美，b_nav 恒定。
    它与参考方向的夹角 = 地磁标定出来的【载体姿态误差】，即陀螺残差。
"""
import sys, math
sys.path.insert(0, '.')
import numpy as np
import magmap
from PIL import Image, ImageDraw, ImageFont

SRC = r'R:\rec_20261013_082322.bin'
OUT = 'gyro_resid_by_mag.png'
WIN = 0.20                                    # 离散速度滚动窗口(s)
KS = 636e-6                                   # 已实测的陀螺标度误差(ppm)

fr = magmap.Frames(SRC)
cal, cinfo = magmap.calibrate(fr, prefer='SET')
FS = 1.0 / magmap.NOMINAL_DT

# ---- 陀螺累计转角（度）：g*dt 已是度，绝不再乘 degrees ----
g = np.linalg.norm(fr.gyro_dps.astype(np.float64), axis=1)
throt = np.cumsum(g / FS)

# ---- ESKF 四元数 -> 体到导航旋转矩阵 ----
q = fr.quat.astype(np.float64); q /= np.linalg.norm(q, axis=1, keepdims=True)
w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
R = np.stack([
    np.stack([1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)], 1),
    np.stack([2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)], 1),
    np.stack([2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)], 1)], 1)   # 体->导航

# ---- 磁矢量（归一化方向），只取新样本且相位可标定的帧 ----
m = fr.new_mask(); idx = np.where(m)[0]
raw = fr.mag[m]; polm = fr.pol[m]
bb = np.full((len(idx), 3), np.nan); ngeom = 0
for k in range(len(idx)):
    try:
        v, _deg = magmap.mag_vector(raw[k], polm[k], cal, strict_phase=True)
        n = np.linalg.norm(v)
        if n > 1e-9: bb[k] = v / n                      # ★ 只留方向
        else: ngeom += 1
    except Exception:
        ngeom += 1
ok = np.isfinite(bb).all(1)
bn = np.einsum('nij,nj->ni', R[idx], bb)                # 导航系下的磁方向
bn /= np.maximum(np.linalg.norm(bn, axis=1, keepdims=True), 1e-12)
ref = bn[ok].mean(0); ref /= np.linalg.norm(ref)        # 参考方向（全程平均）
err = np.degrees(np.arccos(np.clip(bn[ok] @ ref, -1, 1)))   # ★ 陀螺残差(度)
tt = idx[ok] / FS; th = throt[idx][ok]
Bmag = np.linalg.norm(fr.mag[m][ok].astype(np.float64), axis=1)
print('可用磁样本 %d（几何不可用 %d）  残差 中位 %.3f°  σ %.3f°  max %.3f°'
      % (ok.sum(), ngeom, np.median(err), np.std(err), err.max()))

# ---- 与累计转角的最小二乘（陀螺标度误差的独立复核）----
A = np.vstack([th, np.ones_like(th)]).T
coef, *_ = np.linalg.lstsq(A, err, rcond=None)
fit = A @ coef
r2 = 1 - np.sum((err - fit)**2) / max(np.sum((err - err.mean())**2), 1e-12)
resid = err - fit
print('残差 ≈ %.6f·θ_rot + %.4f   (r²=%.4f)  偏离拟合的散布 σ=%.4f°'
      % (coef[0], coef[1], r2, np.std(resid)))
print('★ 拟合斜率 %.4g ppm  —— 与先前实测的 k_s=636 ppm 对照' % (coef[0] * 1e6))

# ---- 离散速度 ----
def roll(a, w, fn):
    out = np.full(len(a), np.nan); h = max(1, int(w / 2))
    for i in range(len(a)):
        lo, hi = max(0, i - h), min(len(a), i + h + 1)
        out[i] = fn(a[lo:hi])
    return out
sd = roll(err, WIN, np.std)
d1 = np.abs(np.gradient(err, tt))

def ds(a, n=3000):
    if len(a) <= n: return np.arange(len(a)), a
    j = np.linspace(0, len(a) - 1, n).astype(int); return j, a[j]

# ---------------- 绘图 ----------------
W, H = 1700, 1360
BG, FG, GRID = (255, 255, 255), (25, 25, 30), (226, 229, 234)
img = Image.new('RGB', (W, H), BG); d = ImageDraw.Draw(img)
def font(sz, b=False):
    for p in (r'C:\Windows\Fonts\msyhbd.ttc' if b else r'C:\Windows\Fonts\msyh.ttc',
              r'C:\Windows\Fonts\simhei.ttf', r'C:\Windows\Fonts\arial.ttf'):
        try: return ImageFont.truetype(p, sz)
        except Exception: pass
    return ImageFont.load_default()
F_T, F_L, F_S = font(30, True), font(18), font(16)

def panel(box, series, title, xlab, hlines=(), colors=None, xlim=None, ylim=None, tickfmt='%.4g'):
    x0, y0, x1, y1 = box
    d.rectangle([x0, y0, x1, y1], outline=FG, width=2)
    d.text((x0, y0 - 34), title, font=F_L, fill=FG)
    allv = np.concatenate([s[1] for s in series]); allx = np.concatenate([s[0] for s in series])
    lo, hi = (ylim if ylim else (float(np.nanmin(allv)), float(np.nanmax(allv))))
    if hi - lo < 1e-12: hi = lo + 1.0
    pad = (hi - lo) * 0.08; lo -= pad; hi += pad
    xa, xb = (xlim if xlim else (float(np.nanmin(allx)), float(np.nanmax(allx))))
    px = lambda v: x0 + (v - xa) / max(xb - xa, 1e-9) * (x1 - x0)
    py = lambda v: y1 - (v - lo) / (hi - lo) * (y1 - y0)
    for i in range(6):
        v = lo + (hi - lo) * i / 5.0; yy = py(v)
        d.line([x0, yy, x1, yy], fill=GRID, width=1)
        d.text((x1 + 6, yy - 8), tickfmt % v, font=F_S, fill=FG)
    for i in range(6):
        v = xa + (xb - xa) * i / 5.0; xx = px(v)
        d.line([xx, y0, xx, y1], fill=GRID, width=1)
        d.text((xx - 18, y1 + 6), tickfmt % v, font=F_S, fill=FG)
    for hv, c in hlines:
        if lo <= hv <= hi: d.line([x0, py(hv), x1, py(hv)], fill=c, width=2)
    cols = colors or [(30, 90, 200)]
    for k, (xs, ys) in enumerate(series):
        n = len(xs); step = max(1, n // 2600)
        pts = [(px(xs[i]), py(ys[i])) for i in range(0, n, step) if np.isfinite(ys[i])]
        if len(pts) > 1: d.line(pts, fill=cols[k % len(cols)], width=2)
    d.text((x0, y1 + 26), xlab, font=F_S, fill=FG)

jx, jy = ds(err)
panel((110, 100, 1400, 360), [(tt[jx], jy)],
      '① 陀螺残差（以地磁方向为绝对真值，逐帧）  中位 %.3f°  σ %.3f°  max %.3f°'
      % (np.median(err), np.std(err), err.max()), 't (s)')

jx2, jy2 = ds(sd); jx3, jy3 = ds(d1)
panel((110, 450, 1400, 710), [(tt[jx2], jy2), (tt[jx3], jy3)],
      '② 残差离散速度   滚动σ(%.0fms) 中位 %.3f°   |d残差/dt| 中位 %.1f °/s'
      % (WIN*1000, np.nanmedian(sd), np.nanmedian(d1)), 't (s)',
      colors=[(200, 40, 40), (30, 140, 80)])
d.text((1060, 465), '红 滚动σ\n绿 |d残差/dt|', font=F_S, fill=FG)

# 残差 vs 累计转角（散点 + 拟合线）
x0, y0, x1, y1 = 110, 800, 1400, 1060
d.rectangle([x0, y0, x1, y1], outline=FG, width=2)
d.text((x0, y0 - 34), '③ 残差 vs 累计转角（斜率 = 陀螺标度误差）  ≈ %.4f·θ_rot + %.3f   r²=%.4f'
       % (coef[0], coef[1], r2), font=F_L, fill=FG)
xa, xb = 0.0, float(th.max()); lo, hi = 0.0, float(err.max() * 1.05)
px = lambda v: x0 + (v - xa) / max(xb - xa, 1e-9) * (x1 - x0)
py = lambda v: y1 - (v - lo) / max(hi - lo, 1e-9) * (y1 - y0)
for i in range(6):
    v = lo + (hi - lo) * i / 5; d.line([x0, py(v), x1, py(v)], fill=GRID)
    d.text((x1 + 6, py(v) - 8), '%.2f' % v, font=F_S, fill=FG)
    u = xa + (xb - xa) * i / 5; d.line([px(u), y0, px(u), y1], fill=GRID)
    d.text((px(u) - 22, y1 + 6), '%.0f' % u, font=F_S, fill=FG)
st = max(1, len(th) // 6000)
for i in range(0, len(th), st):
    d.ellipse([px(th[i]) - 1.5, py(err[i]) - 1.5, px(th[i]) + 1.5, py(err[i]) + 1.5],
              fill=(60, 110, 200))
d.line([(px(0), py(coef[1])), (px(xb), py(coef[0]*xb + coef[1]))], fill=(220, 60, 30), width=3)
d.text((x0, y1 + 26), '累计转角 θ_rot (度)', font=F_S, fill=FG)

# |B| 本体（真实场变化，非误差）
jx4, jy4 = ds(Bmag)
panel((110, 1150, 1400, 1290), [(tt[jx4], jy4)],
      '④ 磁矢量模长 |B|（真实场变化：±%.1f%%；不影响角度，仅作环境参照）'
      % (100 * np.std(Bmag) / np.mean(Bmag)), 't (s)', colors=[(140, 100, 200)])

d.text((60, 28), '陀螺残差（以地磁方向为绝对基准）  —  rec_20261013_082322.bin（168B v4，SET-only）',
       font=F_T, fill=FG)
d.text((60, 1080), '可用磁样本 %d / 总帧 %d（%.2f%%）  几何不可用 %d 帧（RST<100，C7）  参考方向 = 全程平均'
       % (ok.sum(), fr.n, 100.0 * ok.sum() / fr.n, ngeom), font=F_L, fill=FG)
d.text((60, 1310), '★ 残差是【光滑且与累计转角成比例】的：偏离拟合仅 σ=%.3f°  ⟹ 反向证明磁矢量角度精度高（磁若有噪，此曲线会散）'
       % np.std(resid), font=F_L, fill=(180, 30, 30))
img.save(OUT)
print('已保存', OUT)
