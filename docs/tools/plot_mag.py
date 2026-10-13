# -*- coding: utf-8 -*-
"""地磁逐帧探测误差 + 残差离散速度 —— 作图（Pillow）。

严格走 magmap.py 的防错层：
  * 帧解析 / 跳过开头 8192 B / 布局判别     -> magmap.Frames
  * 两相不可混用(C4) / 零点逐录制解(C5)     -> magmap.calibrate
  * 相位强制(C6) / SET-only 回退(C7)        -> mag_vector(strict_phase)
  * 卡死判据(C9)                            -> new_mask()
"""
import sys, math
sys.path.insert(0, '.')
import numpy as np
import magmap
from PIL import Image, ImageDraw, ImageFont

SRC = r'R:\rec_20261013_082322.bin'
OUT = 'mag_error_082322.png'
WIN = 0.10          # 离散速度的滚动窗口(秒)

fr = magmap.Frames(SRC)
FS = 1.0 / magmap.NOMINAL_DT
cal, cinfo = magmap.calibrate(fr, prefer='SET')
print(cal)

# ---- 只取"有新磁样本"的帧（C9），并逐帧过 mag_vector（含相位强制）----
m = fr.new_mask()
idx = np.where(m)[0]
raw = fr.mag[m]
polm = fr.pol[m]
vec = np.full((len(idx), 3), np.nan)
ngeom = 0
for k in range(len(idx)):
    try:
        vec[k] = magmap.mag_vector(raw[k], polm[k], cal, strict_phase=True)[0]
    except Exception:
        ngeom += 1
ok = np.isfinite(vec).all(1)
B = np.linalg.norm(vec, axis=1)                       # Gauss
t = idx / FS
print('新磁样本 %d 帧，几何不可用 %d 帧，可用 %d 帧' % (len(idx), ngeom, ok.sum()))

R = float(cal['SET'].radius_gauss)                          # 拟合半径
res = (B - R) * magmap.MAG_CNT_PER_G                   # 残差(counts)
res_pct = (B - R) / R * 100.0
tt, rr = t[ok], res[ok]

# ---- 残差离散速度：滚动标准差 / 滚动 |d/dt| ----
def roll(x, w, fn):
    out = np.full(len(x), np.nan)
    h = max(1, int(w / 2))
    for i in range(len(x)):
        a, b = max(0, i - h), min(len(x), i + h + 1)
        out[i] = fn(x[a:b])
    return out
sd = roll(rr, WIN, np.std)
d1 = np.abs(np.gradient(rr, tt)) if len(tt) > 2 else np.zeros_like(rr)

def ds(a, n=3000):
    if len(a) <= n: return np.arange(len(a)), a
    j = np.linspace(0, len(a) - 1, n).astype(int)
    return j, a[j]

# ---------------- Pillow 绘图 ----------------
W, H = 1700, 1320
BG, FG, GRID = (255, 255, 255), (25, 25, 30), (226, 229, 234)
img = Image.new('RGB', (W, H), BG)
d = ImageDraw.Draw(img)
def font(sz, b=False):
    for p in (r'C:\Windows\Fonts\msyhbd.ttc' if b else r'C:\Windows\Fonts\msyh.ttc',
              r'C:\Windows\Fonts\simhei.ttf', r'C:\Windows\Fonts\arial.ttf'):
        try: return ImageFont.truetype(p, sz)
        except Exception: pass
    return ImageFont.load_default()
F_T, F_L, F_S = font(30, True), font(18), font(16)

def panel(box, series, title, ylab, xlab, colors=None, hlines=(), yzero=True):
    x0, y0, x1, y1 = box
    d.rectangle([x0, y0, x1, y1], outline=FG, width=2)
    d.text((x0, y0 - 34), title, font=F_L, fill=FG)
    # 数据范围
    allv = np.concatenate([s[1] for s in series])
    allx = np.concatenate([s[0] for s in series])
    lo, hi = float(np.nanmin(allv)), float(np.nanmax(allv))
    if hi - lo < 1e-12: hi = lo + 1.0
    pad = (hi - lo) * 0.08; lo -= pad; hi += pad
    xa, xb = float(np.nanmin(allx)), float(np.nanmax(allx))
    def px(x): return x0 + (x - xa) / max(xb - xa, 1e-9) * (x1 - x0)
    def py(v): return y1 - (v - lo) / (hi - lo) * (y1 - y0)
    # 网格 + 刻度
    for i in range(6):
        v = lo + (hi - lo) * i / 5.0
        y = py(v)
        d.line([x0, y, x1, y], fill=GRID, width=1)
        d.text((x1 + 6, y - 8), '%.4g' % v, font=F_S, fill=FG)
    for i in range(6):
        x = xa + (xb - xa) * i / 5.0
        xx = px(x)
        d.line([xx, y0, xx, y1], fill=GRID, width=1)
        d.text((xx - 16, y1 + 6), '%.1f' % x, font=F_S, fill=FG)
    if yzero and lo < 0 < hi: d.line([x0, py(0), x1, py(0)], fill=(200, 60, 60), width=1)
    for hv, c in hlines:
        if lo <= hv <= hi: d.line([x0, py(hv), x1, py(hv)], fill=c, width=2)
    cols = colors or [(30, 90, 200)]
    for k, (xs, ys) in enumerate(series):
        n = len(xs); step = max(1, n // 2600)
        pts = [(px(xs[i]), py(ys[i])) for i in range(0, n, step) if np.isfinite(ys[i])]
        if len(pts) > 1: d.line(pts, fill=cols[k % len(cols)], width=2)
    d.text((x0, y1 + 26), xlab, font=F_S, fill=FG)
    d.text((x0 - 6, y0 - 58), ylab, font=F_S, fill=FG)

# 面板1: |B| 逐帧 + 拟合半径
jx, jy = ds(B[ok])
panel((110, 100, 1420, 380), [(tt[jx], jy)], 
      '① 地磁逐帧探测：|B| 逐帧（新磁样本）', '|B| (G)', 't (s)',
      hlines=[(R, (220, 120, 30))])
d.text((1090, 115), '拟合半径\n%.4f G' % R, font=F_S, fill=(220, 120, 30))

# 面板2: 残差（counts）
jx2, jy2 = ds(rr)
panel((110, 480, 1420, 760), [(tt[jx2], jy2)],
      '② 逐帧残差 = |B| − R（counts）   散度 %.3f%%' % (np.std(rr) / (R * magmap.MAG_CNT_PER_G) * 100),
      '残差 (counts)', 't (s)')

# 面板3: 残差离散速度（滚动标准差 + |d/dt|）
jx3, jy3 = ds(sd); jx4, jy4 = ds(d1)
panel((110, 860, 1420, 1140), [(tt[jx3], jy3), (tt[jx4], jy4)],
      '③ 残差离散速度', 'counts / counts·s⁻¹', 't (s)',
      colors=[(200, 40, 40), (30, 140, 80)])
d.text((1090, 875), '红 滚动σ(%.0fms)\n绿 |d残差/dt|' % (WIN * 1000), font=F_S, fill=FG)

d.text((60, 30), '地磁逐帧误差与残差离散速度  —  rec_20261013_082322.bin（168B v4，SET-only）', font=F_T, fill=FG)
d.text((60, 1180), 
       '新磁样本 %d / 总帧 %d（%.2f%%）；几何不可用 %d 帧；拟合半径 %.4f G = %.1f counts' %
       (ok.sum(), fr.n, 100.0 * ok.sum() / fr.n, ngeom, R, R * magmap.MAG_CNT_PER_G),
       font=F_L, fill=FG)
d.text((60, 1212),
       '残差：中位 %.1f counts（%.3f%%）   σ %.1f counts（%.3f%%）   峰峰 %.1f counts（%.3f%%）' %
       (np.median(rr), np.median(res_pct[ok]), np.std(rr), np.std(rr) / (R * magmap.MAG_CNT_PER_G) * 100,
        rr.max() - rr.min(), (rr.max() - rr.min()) / (R * magmap.MAG_CNT_PER_G) * 100),
       font=F_L, fill=FG)
d.text((60, 1244),
       '离散速度：滚动σ 中位 %.1f counts / 95%% %.1f / max %.1f     |d/dt| 中位 %.1f counts·s⁻¹ / 95%% %.1f' %
       (np.nanmedian(sd), np.nanpercentile(sd, 95), np.nanmax(sd),
        np.nanmedian(d1), np.nanpercentile(d1, 95)),
       font=F_L, fill=FG)
d.text((60, 1276),
       'C4 分相不可混用 / C5 零点逐录制解 / C7 RST<100 ⟹ SET-only  —  本图未做任何跨相或跨录制复用',
       font=F_S, fill=(90, 90, 100))
img.save(OUT)
print('已保存', OUT)
print('残差 中位 %.2f  σ %.2f  p2p %.2f counts' % (np.median(rr), np.std(rr), rr.max() - rr.min()))
print('滚动σ 中位 %.2f  95%% %.2f  max %.2f' % (np.nanmedian(sd), np.nanpercentile(sd, 95), np.nanmax(sd)))
print('|d/dt| 中位 %.2f  95%% %.2f' % (np.nanmedian(d1), np.nanpercentile(d1, 95)))
