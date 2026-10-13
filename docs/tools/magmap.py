# -*- coding: utf-8 -*-
"""magmap —— 地磁原始 counts → 地磁矢量（体坐标）的映射函数 + 状态机。

===============================================================================
本文件是【本轮全部已核实结论的固化】。每条硬约束后面括注的是它的实测依据
（数据来自 _recordings / R:\\ / _baseline_pc_chain_20261012\\data 的原始 .bin，
  与生成它们的固件源码）。凡与本文冲突的 md 文档，一律不作依据。
===============================================================================

硬约束（不得违反）
------------------
C1 帧解析必须先跳过文件开头 8192 B。
   [依据] USB/缓冲排空；不跳会得到错误间距与错位字段。

C2 84 B 帧有两种长度相同、type 同为 2.0 的布局，必须从字节判别：
     RAW 布局: 磁在 col10..12, mmc_cnt=col16, pol=col17, ts=col13/14, col19/20 ≡ 0
     DBG 布局: 磁在 col14..16, ts=col17/18, imu_cnt=col19, mmc_cnt=col20
   判别式：col19 与 col20 是否恒为 0（恒 0 ⟹ RAW）。
   [依据] justfloat.c 的 jf_raw_push()/jf_dbg_push() 同为 84 B，type 均 2.0f。

C3 120 B 帧: 磁原始 counts 在 col23..25(当前拍) / col26..28(上一拍)，
   flag = col17, flag2 = col29（bit0 pol_cur, bit1 pol_prev, bit2 prev_valid, bit3 ready），
   mmc_cnt = col20, imu_cnt = col19。col14..16 是 mag_u，本版固件写死为 0，不可用。
   [依据] justfloat.c 的 jf_full_push() 逐字段顺序 + main.c:186 实参。

C4 ★ 绝不允许把 SET 相与 RESET 相当成同一个测量的两次采样，
   也绝不允许用 (m_SET + m_RESET)/2 当运行零点。
   [依据] 实测两相是两个【不同心的球】：半径 8316.6 vs 8320.5（比 0.99953），
          球心相距 |Δ| = 3588.2 counts = 场强的 43.15%，由两条独立路径互证到 0.1 counts。
          混用 ⟹ 单球拟合 12.95% 散度、最佳椭球仍有 11.87%（几何下限 10.76%）；
          用中点当零点 ⟹ 留下 |Δ|/2 = 1794 counts = 21.6% 的体坐标固定偏置
          ⟹ 模长出现峰峰 42.1% 的确定性方向调制（曾被误读为"±20% 三轴增益不平衡"）。
          分相各自减自己的球心 ⟹ 残差 0.678% / 0.700%。

C5 零点必须【逐录制】解，不能沿用常量。
   [依据] SET 相球心跨 6 份录制: x 跨度 38 counts、y 跨度 204 counts
          （≈ 水平半径 4031 的 5%）。

C6 偏航（单轴）录制对【转轴方向的分量】先天不可辨识：
   圆拟合沿转轴退化（任意沿轴的球心滑动都给出同一残差）。
   ⟹ 此时输出的【倾角/竖直分量】不可信，必须由 axis_known=False 标出。
   [依据] 球心可沿轴任意滑动而 |m−c| 恒定；线性解中 o_z 与 B0_z 不可分辨。

C7 RESET 相样本不足（< MIN_RST）时，只能走 SET-only。
   [依据] 固件改为"每 1024 个 SET 一次 RESET"后，实测 RST 仅 11~45 个/录制，
          无法自标定；而 SET-only + SET 相球心是可行且稳定的路径。

C8 加计若来自 ±2g 器件却按 ±16g 常数换算，上报 |a| ≈ 8.04，必须 ÷8 才是 g。
   [依据] 0.488/0.061 = 8.000；实测该批 |a| 中位 8.041~8.053，÷8 后 1.0051~1.0066，
          与 ±16g 批次（0.988~1.009）一致。
   ★ 该错误还会静默关闭上报流里的陀螺零偏追踪（proc_attitude_align 门槛不通过），
     所以陀螺须走 corrected_dps() 补追踪。

C9 有效性判据：新磁样本 = mmc_cnt 递增；卡死 = 连续 MAG_STALL 帧无新样本。
   [依据] 历史上 MMC 多次中途冻结；冻结时 imu/bmp 仍正常。

换算常量（器件手册）
------------------
MAG_CNT_PER_G = 16384.0（18 位，±8 G 满量程）；零点名义 131072。
[校验] 六面全姿态数据 SET 相球拟合 R = 8316.6 counts ⟹ 0.5077 G，与地磁场量级相符。
"""
import numpy as np

MAG_CNT_PER_G = 16384.0
NOMINAL_ZERO = 131072.0
NOMINAL_DT = 1.0 / 7673.5
MAG_STALL_FRAMES = 1536          # 与固件 proc_tune.h 一致（约 200 ms @7.68 kHz）
MIN_RST = 100                    # RESET 相可用所需最少样本（C7）
AXIS_COV_RATIO = 0.05            # 协方差最小/最大特征值比；低于此判"转轴分量不可辨识"（C6）
COV_MIN_DEG = 60.0               # ★ 标定所需最小覆盖半角（E20）
RING_BYTES = 8192                # ★ MCU 侧 CDC 环形缓冲大小（E22）

# ---------------------------------------------------------------------------
# 阻断清单：本模块存在的【首要目的】是让使用者没有机会犯下这些错误。
# 每一条都是本轮真实发生过的（含我自己犯的），并给出了曾造成的假象。
#   E1  不跳开头 8192 B                    -> 错误帧间距与错位字段
#   E2  用 DBG 布局解析 RAW 的 84 B 文件      -> 把 gyro_z/accel_x/accel_y 当成"磁"
#   E3  把 92 B 旧格式与 84/120 混用         -> 全字段错位
#   E4  用 (m_SET+m_RESET)/2 中点当零点      -> 模长峰峰 42.1% 的假各向异性
#   E5  两相样本混进同一个球/椭球拟合         -> 最佳椭球仍残差 11.87%（无法拟合）
#   E6  跨录制复用同一个零点                 -> y 差 204 counts ≈ 水平半径的 5%
#   E7  把偏航录制的球拟合当作定出了三维球心  -> 转轴分量任意，倾角不可信
#   E8  用 120B 的 col14..16 当磁场          -> 那是 mag_u，本版固件写死 0
#   E9  把 84B 的原始 counts 当成已是 G/dps   -> 量纲错 16384 倍 / 0.140 倍
#   E10 不修 ±2g 批次的加计(÷8)与陀螺零偏     -> |a|=8.04；陀螺漂 37°/45s
#   E11 把被旁路链路的四元数当有效             -> 它恒为 (1,0,0,0)
#   E12 拿 atan2 的 yaw 直接做最小二乘         -> ±180 环绕 + 非单调序列 ⟹ 残差 87°
#   E13 用"|gyro| 最小的 5% 帧的均值"估零偏    -> 选择偏置把估计值向 0 收缩
#   E14 椭球拟合用 rhs=1 的代数形式            -> 半轴比 1.5~3.5e9 的胡说
#   E15 拿陀螺当基准去判磁                    -> 循环论证（陀螺自身标度误差 0.35~0.83%）
#   E16 用 t_us 算时长                       -> 部分录制给出 37068 s 的荒谬时长
#   E17 调用名字含 unit 的函数却不真的归一化    -> 点积被 clip 到 1 ⟹ arccos 全给 0°，
#                                              把 0.47° 的误差伪装成 0.000°
#   E18 把偏航录制的球拟合半径当成 |B|          -> 那只是【垂直转轴的分量】：实测 0.247 G
#                                              而对全场强 0.508 G，会得出"场强少一半"
#   E19 平台窗口跨过 USB 丢帧的缺口             -> 把缺口两侧不同时刻的样本平均到一起，
#                                              伪造出"同停位散布"（实测 0.73° -> 0.28°）
#                                              并把丢失的运动段整段吞掉（实测丢 11.5 s）
# ---------------------------------------------------------------------------
class MagMapError(RuntimeError):
    pass


class ForbiddenConvention(MagMapError):
    """使用者试图使用已被证伪的口径。"""
    pass


class GeometryError(MagMapError):
    """样本几何不支持所请求的量（见 C6）。"""
    pass


class LayoutError(MagMapError):
    """帧布局不明确或不支持。"""
    pass

# 状态机的状态
ST_INIT = 0      # 未拿到任何校准量
ST_CALIB = 1     # 正在累积样本、拟合零点
ST_READY = 2     # 已校准，正常出矢量
ST_STALL = 3     # 磁通道卡死（无新样本）
ST_BAD = 4       # 输出不可信（几何退化 / 相位不匹配 / 无有效相）


# ============================================================================
# 帧解析：从 .bin 得到规范化的帧矩阵 + 格式信息
# ============================================================================
def _load_raw_frames(path):
    """按 C1/C2/C3 解析。返回 (F, step, note)。

    ★ E21：不能用"首个 sync + 固定步长"整块 reshape。
      实测 USB 断连会在拼接处留下【半帧】：rec_20261013_041511 全流 94,075 帧中
      只有 1 处间距是 176（=120+56），其余全部 120。固定步长解析器会被这 56 字节
      整体错位，静默丢掉 68,056 帧（72%）而不报错。
      正确做法：先收集所有 sync 对位置，按"间距==众数"切成连续段，逐段解析，
      并把断点数量与位置显式报出来。
    """
    b = open(path, 'rb').read()
    SY = bytes([0, 0, 0x80, 0x7F])
    pat = SY * 2
    pos = []
    i = 8192                                     # C1
    find = b.find
    while True:
        j = find(pat, i)
        if j < 0:
            break
        pos.append(j)
        i = j + 4                                # 跳过本 sync 对，找下一个
    if len(pos) < 4:
        raise LayoutError('找不到足够的 sync 对（已跳过 8192 B）: %d' % len(pos))
    pos = np.asarray(pos, dtype=np.int64)
    sp = np.diff(pos)
    vals, cnts = np.unique(sp, return_counts=True)
    step = int(vals[int(np.argmax(cnts))])       # 众数间距 = 帧长
    if step not in (84, 92, 120, 132, 164, 168):
        raise LayoutError('非法的帧长间距 %d（只支持 84/92/120/132/164/168）' % step)
    if step == 92:
        raise LayoutError('92 B 旧格式布局不同，绝不可与 84/120/132/164 混用')

    brk = np.where(sp != step)[0]                # 第 k 个间距异常 ⟹ 第 k+1 帧是新区段起点
    starts = np.concatenate([[0], brk + 1])
    ends = np.concatenate([brk, [len(pos) - 1]])
    nfl = step // 4
    mats = []
    seg_id = []
    for k, (s, e) in enumerate(zip(starts, ends)):
        nfr = int(e - s + 1)
        o = int(pos[s])
        mats.append(np.frombuffer(b, dtype='<f4', count=nfr * nfl, offset=o).reshape(-1, nfl))
        seg_id.append(np.full(nfr, k, dtype=np.int32))
    F = np.concatenate(mats).astype(np.float64)
    seg_id = np.concatenate(seg_id)
    keep = np.abs(F[:, 2] - 2.0) < 1e-6
    bad = int((~keep).sum())
    F = F[keep]
    seg_id = seg_id[keep]

    # ★ 每段（= 一次 CDC 恢复）开头的 RING_BYTES 是【停机期间积压的陈旧数据】。
    #   MCU 侧：CDC 受阻时继续写 8 KB 环形缓冲，写满后厂家的安全 API
    #   丢弃其后所有写入请求；恢复后先把这 8 KB 吐出来，再接实时数据。
    #   ⟹ 这 8 KB 与之后的数据【时序不连续】，属正常现象，分析中必须排除。
    seg_starts = []
    for k in range(1, len(starts)):
        w = np.where(seg_id == k)[0]
        if len(w):
            seg_starts.append(int(w[0]))

    note = ''
    if step == 84:                               # C2
        if np.all(F[:, 19] == 0.0) and np.all(F[:, 20] == 0.0):
            note = 'RAW 布局（col19/20 ≡ 0）'
        else:
            raise LayoutError('84 B 文件是 DBG 布局（col19/20 非零）—— 本模块只实现 RAW')
    if len(brk):
        note += ('；★ 流内有 %d 处错位断点，已按段解析；每段开头 %d B 为 CDC 积压陈旧段（已标 stale）'
                 % (len(brk), RING_BYTES))
    if bad:
        note += '；★ %d 帧 type 校验失败（可能含半帧）' % bad
    return F, step, note, seg_starts


class Frames(object):
    """一次解析，按长度暴露统一字段（C2/C3）。"""

    def __init__(self, path):
        self.F, self.step, self.note, seg_starts = _load_raw_frames(path)
        F, st = self.F, self.step
        # ★ E22：每段开头 RING_BYTES 是 CDC 积压的陈旧数据，与之后时序不连续。
        #   【第 0 段也要标】：启动后上位机尚未连接时，MCU 同样处于 CDC 受阻状态，
        #   所以文件最开头的 8 KB 也是连接前积压的陈旧数据（实测它会让
        #   帧计数出现 66,193 帧 = 8.63 s 的跳变，看起来像"丢了一大段"）。
        self.stale = np.zeros(len(F), bool)
        self.n_stale_frames = int(np.ceil(RING_BYTES / float(st)))
        for s in [0] + list(seg_starts):
            self.stale[s:s + self.n_stale_frames] = True
        if st == 84:
            self.mag = F[:, 10:13]
            self.pol = F[:, 17].astype(int)
            self.mmc_cnt = F[:, 16]
            self.imu_cnt = F[:, 15]
            self.gyro_lsb = F[:, 4:7]
            self.gyro_dps = F[:, 4:7] * 0.140
            self.accel_raw = F[:, 7:10]
            self.accel_g = F[:, 7:10] * 0.000488          # 该代为 ±16g 口径
            self.acc_mode = 'raw_counts_16g'
            self.temp = F[:, 18]
        else:
            self.mag = F[:, 23:26]
            self.mag_prev = F[:, 26:29]
            self.flag2 = F[:, 29].astype(int)
            self.pol = self.flag2 & 1
            self.mmc_cnt = F[:, 20]
            self.imu_cnt = F[:, 19]
            self.gyro_dps = F[:, 8:11]
            self.accel_g = F[:, 11:14]
            an = np.linalg.norm(self.accel_g, axis=1)
            self.acc_mode = 'g_x8' if abs(float(np.median(an)) - 8.0) < 1.2 else 'g_ok'
            if self.acc_mode == 'g_x8':
                self.accel_g = self.accel_g / 8.0          # C8
            self.temp = None
        self.n = len(F)

        # ---- 防错：时间基准（E16）----
        d = np.diff(self.imu_cnt.astype(float))
        d = np.where(d <= 0.0, 1.0, d)
        d = np.clip(d, 1.0, 1.0e6)
        # ★ 缺口感知（E19）：USB 瞬时接触不良会造成【整块丢帧】。
        #   gap[i] = 第 i 帧之前缺失的采样数；任何平台检测都必须在缺口处断开，
        #   否则会把缺口两侧不同时刻的样本平均到一起，伪造出"同停位散布"。
        self.gap = np.concatenate([[0.0], np.where(d > 1.0, d - 1.0, 0.0)])
        dm = np.diff(self.mmc_cnt.astype(float))
        self.mag_gap = np.concatenate([[0.0], np.where(dm > 1.0, dm - 1.0, 0.0)])
        # ★ E26：CDC 积压陈旧段的计数跳变可达 1e5~1e6 帧，若计入会把 duration_s
        #   撑大数倍（实测 rec_045027：真实 45 s，被算成 175.3 s），进而让
        #   "路径长度/时长"类判据全部失真。超过 JUMP_MAX 的跳变判为计数假象，
        #   不计入时长，并单独记账。
        JUMP_MAX = 1.0e5
        big = d > JUMP_MAX
        self.n_counter_jump = int(big.sum())
        self.counter_jump_frames = float((d[big] - 1.0).sum()) if big.any() else 0.0
        self.duration_s = float(np.where(big, 1.0, d).sum() * NOMINAL_DT)
        self.time_source = ('imu_cnt 差 × 名义帧间隔 %.6f s'
                            '（不用 t_us：部分录制上它给出 37068 s 级荒谬时长，见 E16）' % NOMINAL_DT)

        # ---- 防错：四元数可用性（E11）----
        # ★ 布局版本：v1 = 120 B；v2 = 132 B（尾部追加 [30..32] = gyro_bias_dps[3]）。
        #   纯尾部追加 ⟹ v2 的前 30 个 float 与 v1 偏移完全一致，老字段照读。
        #   为什么要上报零偏：运动期零偏在陀螺单源下不可观测，
        #   冻结期间的 held bias 与运动前后收敛值之差，是判定
        #   "运动前估计不准" 还是 "运动期真的游走" 的唯一直接证据（见 E27）。
        self.layout_v = 4 if st == 168 else (3 if st == 164 else (2 if st == 132 else 1))
        self.gyro_bias_dps = F[:, 30:33] if st in (132, 164, 168) else None
        # ★ v3 = 164 B：逐帧携带 ESKF 持久状态（历史量，截取数据无法反推）
        #   [33] acc_thr  自适应重力残差门限——直接决定 grav_open
        #   [34..39] P 的 6 个独立分量 Pxx,Pyy,Pzz,Pxy,Pxz,Pyz
        #            ★ 非对角项就是"重力更新经互协方差漏到偏航"的通道
        #   [40] calib_n  启动标定累计样本数
        self.acc_thr = F[:, 33] if st in (164, 168) else None
        self.p_sym = F[:, 34:40] if st in (164, 168) else None
        self.calib_n = F[:, 40] if st in (164, 168) else None
        if st in (120, 132, 164, 168):
            q = F[:, 4:8]
            if float(np.abs(q - q[0]).max()) <= 1e-6:
                self.quat = None
                self.quat_reason = ('四元数恒定于初值：上报链被旁路'
                                    '（加计标度错 8× ⟹ proc_attitude_align 门槛不通过），不可用')
            else:
                self.quat = q
                self.quat_reason = ''
        else:
            self.quat = None
            self.quat_reason = '84 B RAW 帧不含四元数'

    def new_mask(self):
        """C9：新磁样本掩码。"""
        m = np.zeros(self.n, bool)
        m[1:] = np.diff(self.mmc_cnt) > 0
        return m

    # ------------------------------------------------------------------
    def gap_report(self, verbose=True):
        """缺口报告（E19）：USB 瞬时接触不良的诊断。

        判据（沿用基线验收口径）：
          · 缺帧率应 < 5%
          · 缺帧应为【整块丢失】而不是散落逐帧丢失
          · 缺口若落在运动段内，会威胁转台网格归属（丢掉半格就归错）
        """
        d = np.diff(self.imu_cnt.astype(float))
        d = np.where(d <= 0.0, 1.0, d)
        miss = float(self.gap.sum())
        # ★ E22：陈旧段内的计数跳变是 CDC 积压造成的，不是真丢帧，排除
        miss_real = float(self.gap[~self.stale].sum())
        tot = float(self.n + miss) - float(self.stale.sum())
        g = np.where(self.gap > 1.0, self.gap, 0.0)
        g[self.stale] = 0.0
        gi = np.where(g > 0.0)[0]
        blocks = g[gi]
        info = dict(n=self.n, missing=miss_real, missing_raw=miss,
                    miss_pct=100.0 * miss_real / max(tot, 1.0),
                    stale=int(self.stale.sum()),
                    n_block=len(blocks), biggest=float(blocks.max()) if len(blocks) else 0.0,
                    biggest_s=float(blocks.max() * NOMINAL_DT) if len(blocks) else 0.0,
                    block_sum=float(blocks.sum()))
        if verbose:
            print('  缺口: 缺 %d 帧 / 有效 %.0f (%.3f%%)；整块 %d 处，最大 %d 帧 (%.2f s)'
                  % (miss_real, tot, info['miss_pct'], info['n_block'],
                     info['biggest'], info['biggest_s']))
            if info['stale']:
                print('        （另排除 CDC 积压陈旧段 %d 帧；原始缺口计数 %.0f 帧）'
                      % (info['stale'], miss))
        return info


# ============================================================================
# 标定：分相球心（C4/C5/C6/C7）
# ============================================================================
class PhaseCal(object):
    """单相标定：球心 + 半径 + 残差 + 几何可辨识性。"""

    def __init__(self, phase, center, radius, resid_pct, axis_known, n, cov_ratio,
                 coverage_deg=0.0):
        self.phase = phase
        self.center = np.asarray(center, float)
        self.radius = float(radius)
        self.resid_pct = float(resid_pct)
        self.axis_known = bool(axis_known)
        self.n = int(n)
        self.cov_ratio = float(cov_ratio)
        self.coverage_deg = float(coverage_deg)

    @property
    def usable_for_calibration(self):
        """★ E20：这一相能否用于【标定】。

        近静止录制会让 cov_ratio 假阳性（各向同性噪声 → 比值≈1），
        必须另用"覆盖半角"判定：样本方向若只占一个小锥面，球心/半径不可用。
        实测 rec_20261013_041216 是近静止，|B| 被解成 0.043 G（应 0.51 G）。
        """
        return self.axis_known and self.coverage_deg >= COV_MIN_DEG

    def to_gauss(self, m):
        """实测地磁矢量（G，体坐标）。模长保留逐样本信息。

        RST 相内部取反号（手册模型 m_S = +B + b, m_R = -B + b）。
        支持单个 (3,) 或一批 (N,3)。
        """
        v = (np.asarray(m, float) - self.center) / MAG_CNT_PER_G
        return -v if self.phase == 'RST' else v

    @property
    def radius_gauss(self):
        return self.radius / MAG_CNT_PER_G

    def to_unit(self, m):
        """【保证】|·| = 1 的方向矢量。

        ★ E17：本方法曾写成 (m−center)/radius —— 名字叫 unit 却返回模长
          1.000063 的矢量，导致点积中位数 1.0002 > 1，被 clip 到 1 后
          arccos 全部返回 0°，把"0.47° 的方向误差"伪装成"0.000°"。
          凡名字含 unit 的函数必须真的归一化。
        """
        v = self.to_gauss(m)
        n = np.linalg.norm(v, axis=-1, keepdims=True)
        return v / np.maximum(n, 1e-30)

    def mag_scatter(self, m):
        """逐样本 |B| 相对标定半径的散布比（1.0 = 完全一致）。"""
        g = self.to_gauss(m)
        return np.linalg.norm(g, axis=-1) / self.radius_gauss

    @property
    def radius_is_full_field(self):
        """★ 标定半径是否等于【全场强】。

        axis_known=False（偏航/单轴覆盖）时，球拟合半径只是【垂直于转轴的分量】
        —— 本次实测 rec_234513 为 0.247 G，而全场强是 0.508 G。
        把 0.247 当 |B| 会得出"场强少了一半"的荒谬结论，故必须显式区分（E18）。
        """
        return self.axis_known

    def __repr__(self):
        return ('PhaseCal(%s n=%d R=%.1f 残差=%.3f%% 转轴可辨识=%s cov比=%.4f)'
                % (self.phase, self.n, self.radius, self.resid_pct,
                   self.axis_known, self.cov_ratio))


def fit_phase(phase, P):
    """代数球拟合 + 几何可辨识性判定（C6）。

    ★ cov_ratio = 最小/最大 特征值（不是最大/最小！）
      球形覆盖 -> 三轴相当 -> cov_ratio ≈ 1        ⟹ 转轴分量可辨识
      圆/平面覆盖 -> 一个方向几乎无散布 -> cov_ratio ≪ 1 ⟹ 不可辨识
      [实测] rec_031605(六面) 0.749 ⟹ 可辨识；
             rec_234513(纯偏航) 3.1e-5、[7] 4.4e-5 ⟹ 不可辨识
    """
    P = np.asarray(P, float)
    if len(P) < 10:
        raise ValueError('%s 相样本不足 (%d)' % (phase, len(P)))
    A = np.column_stack([2 * P[:, 0], 2 * P[:, 1], 2 * P[:, 2], np.ones(len(P))])
    sol, *_ = np.linalg.lstsq(A, (P ** 2).sum(axis=1), rcond=None)
    c = sol[:3]
    R = float(np.sqrt(max(sol[3] + c @ c, 0.0)))
    r = np.linalg.norm(P - c, axis=1) - R
    Q = P - P.mean(axis=0)
    ev = np.linalg.eigvalsh(Q.T @ Q / len(Q))          # 升序
    ratio = float(ev[0] / ev[-1]) if ev[-1] > 0 else 0.0
    # ★ 覆盖半角（E20）：单位方向的平均模长 |μ| 与锥面半角一一对应
    #   均匀球面锥（半角 θ）: |μ| = (1+cosθ)/2  ⟹  θ = arccos(2|μ|-1)
    #   全姿态 θ=180°；半球 θ=90°；近静止小簇 θ→0°
    U = (P - c)
    nu = np.linalg.norm(U, axis=1)
    U = U[nu > 1e-9] / nu[nu > 1e-9, None]
    mu = float(np.linalg.norm(U.mean(axis=0))) if len(U) else 1.0
    cov_deg = float(np.degrees(np.arccos(np.clip(2.0 * mu - 1.0, -1.0, 1.0))))
    return PhaseCal(phase, c, R, 100.0 * r.std() / R, ratio >= AXIS_COV_RATIO, len(P),
                    ratio, cov_deg)


def calibrate(fr, prefer='AUTO'):
    """从一份录制解出标定。返回 (cal_dict, info)。

    C4: 分相独立球心。C7: RST 不足则只用 SET。
    """
    newm = fr.new_mask()
    idx = np.where(newm)[0]
    p = fr.pol[idx]
    P0 = fr.mag[idx][p == 0]
    P1 = fr.mag[idx][p == 1]
    info = dict(n_set=len(P0), n_rst=len(P1))
    cal = {}
    if len(P0) >= 10:
        cal['SET'] = fit_phase('SET', P0)
    use_rst = len(P1) >= MIN_RST
    if use_rst:
        cal['RST'] = fit_phase('RST', P1)
    info['rst_usable'] = use_rst
    if use_rst and 'SET' in cal:
        d = cal['SET'].center - cal['RST'].center
        info['two_phase_sep'] = float(np.linalg.norm(d))
        info['radius_ratio'] = cal['SET'].radius / cal['RST'].radius
    # 主相：优先 SET（C7）
    order = ['SET'] if 'SET' in cal else ['RST']
    if use_rst and 'RST' in cal and 'SET' in cal and prefer == 'RST':
        order = ['RST', 'SET']
    info['primary'] = order[0]
    return cal, info


# ============================================================================
# 映射函数
# ============================================================================
def mag_vector(raw_counts, pol, cal, strict_phase=True):
    """核心映射：原始 counts + 极性 -> 地磁矢量（G，体坐标）。

    只用【该极性自己那一相】的球心（C4）。若该相未标定：
      strict_phase=True  -> 抛错（防止静默退化成单零点口径）
      strict_phase=False -> 退化为主相球心，并在返回值里标注降级
    """
    raw_counts = np.asarray(raw_counts, float)
    want = 'RST' if int(pol) != 0 else 'SET'
    if want in cal:
        return cal[want].to_gauss(raw_counts), False
    if strict_phase:
        raise KeyError('缺少 %s 相标定（禁止用另一相的球心顶替，见 C4）' % want)
    alt = cal['SET'] if 'SET' in cal else cal['RST']
    return alt.to_gauss(raw_counts), True


def normalized(raw_counts, pol, cal, strict_phase=True):
    v, deg = mag_vector(raw_counts, pol, cal, strict_phase)
    return v, deg


# ============================================================================
# 状态机
# ============================================================================
class MagSolver(object):
    """流式状态机：逐帧喂入，输出地磁矢量与状态。

    状态迁移
        INIT  --有磁样本--> CALIB --样本足够且标定成功--> READY
        READY --连续 MAG_STALL_FRAMES 帧无新样本--> STALL --有新样本--> READY
        任意  --几何退化/相位缺失--> BAD（可用 reset 回 INIT 重新标定）

    典型用法（离线一次性喂入整份录制）:
        fr = Frames(path); cal, info = calibrate(fr)
        sol = MagSolver(cal); sol.run(fr)
    """

    def __init__(self, cal, strict_phase=True, stall_frames=MAG_STALL_FRAMES):
        self.cal = cal
        self.strict_phase = strict_phase
        self.stall_frames = stall_frames
        self._reset_state()

    def _reset_state(self):
        self.state = ST_CALIB if self.cal else ST_INIT
        self.last_cnt = None
        self.stall = 0
        self.n_out = 0
        self.n_skipped = 0
        self.last_vec = None
        self.last_unit = None
        self.degraded = False

    def push(self, raw_counts, pol, mmc_cnt):
        """喂一帧。返回 (state, unit_vec or None, gauss_vec or None)。"""
        if self.last_cnt is None or mmc_cnt != self.last_cnt:
            self.last_cnt = mmc_cnt
            self.stall = 0
            try:
                g, deg = mag_vector(raw_counts, pol, self.cal, self.strict_phase)
                self.degraded = self.degraded or deg
                u = g / max(np.linalg.norm(g), 1e-30)
                self.last_vec, self.last_unit = g, u
                self.n_out += 1
                codes = [c for c in self.cal if c in ('SET', 'RST')]
                self.state = ST_READY if (u is not None) else ST_CALIB
                self.avail = codes
            except KeyError:
                self.n_skipped += 1
                self.state = ST_BAD
                return self.state, None, None
        else:
            if self.stall < 0xFFFFFFFF:
                self.stall += 1
            if self.stall >= self.stall_frames:
                self.state = ST_STALL
                return self.state, None, None
        return self.state, self.last_unit, self.last_vec

    def run(self, fr):
        """离线跑完整份录制。返回结果字典。"""
        newm = fr.new_mask()
        idx = np.where(newm)[0]
        U = np.full((len(idx), 3), np.nan)
        V = np.full((len(idx), 3), np.nan)
        S = np.zeros(len(idx), np.int8)
        for k, i in enumerate(idx):
            st, u, v = self.push(fr.mag[i], fr.pol[i], fr.mmc_cnt[i])
            S[k] = st
            if u is not None:
                U[k], V[k] = u, v
        good = ~np.isnan(U[:, 0])
        return dict(idx=idx, unit=U, gauss=V, state=S, good=good,
                    n_samples=len(idx), n_out=int(good.sum()),
                    peak_g=float(np.nanmedian(np.linalg.norm(V[good], axis=1))) if good.any() else 0.0,
                    degraded=self.degraded, cal=self.cal)


# ============================================================================
# 验收
# ============================================================================
ACCEPT = [
    # (名称, 判据, 阈值, 说明与实测依据)
    ('SET 相球拟合残差', '%', 1.5, '实测 0.48~1.30%'),
    ('RST 相球拟合残差', '%', 1.5, '实测 0.70~0.81%'),
    ('两相半径比 R_S/R_R', 'ratio', 0.03, '实测 0.99953 / 1.00832，容差 ±3%'),
    ('正确口径 |B| 散度(逐平台)', '%', 0.60, '实测 0.499%'),
    ('中点零点口径 |B| 峰峰', '%', 5.0, '反例：实测 42.1% ⟹ 必须远超阈值才算"口径错误被避免"'),
    ('零点分段稳定性 std', 'counts', 10.0, '(S+R)/2 跨静止段 std 实测 (1.5,4.1,4.0)'),
]


def accept(path, verbose=True):
    """跑验收。返回 (all_pass, rows)。"""
    fr = Frames(path)
    cal, info = calibrate(fr)
    sol = MagSolver(cal)
    res = sol.run(fr)
    rows = []

    def add(name, val, thr, ok, note):
        rows.append((name, val, thr, (None if ok is None else bool(ok)), note))

    if 'SET' in cal:
        add('SET 相球拟合残差', cal['SET'].resid_pct, 1.5, cal['SET'].resid_pct <= 1.5, '')
    if 'RST' in cal:
        add('RST 相球拟合残差', cal['RST'].resid_pct, 1.5, cal['RST'].resid_pct <= 1.5, '')
    if 'radius_ratio' in info:
        rr = info['radius_ratio']
        add('两相半径比 R_S/R_R', rr, 0.03, abs(rr - 1.0) <= 0.03, '')
    # 正确口径的 |B| 散度（逐样本，用显式的 mag_scatter，不用 to_unit 的模长）
    newm = fr.new_mask(); idx = np.where(newm)[0]
    prim = info['primary']
    Pm = fr.mag[idx][(fr.pol[idx] == 0) if prim == 'SET' else (fr.pol[idx] == 1)]
    sm = cal[prim].mag_scatter(Pm)
    add('正确口径 |B| 散度(逐样本)', 100 * sm.std() / sm.mean(), 1.0,
        (100 * sm.std() / sm.mean()) <= 1.0, '用分相球心')
    # 反例：两相混用 + 中点零点
    if 'RST' in cal:
        allP = fr.mag[idx]
        cmid = (cal['SET'].center + cal['RST'].center) / 2.0
        gm = np.linalg.norm(allP - cmid, axis=1)
        pp = 100 * (np.percentile(gm, 99.5) - np.percentile(gm, 0.5)) / gm.mean()
        add('中点零点口径 |B| 峰峰（反例）', pp, 5.0, pp >= 5.0, '必须 ≥5% 才证明该口径确实错')
    else:
        add('RST 样本数（<100 则不参与）', info['n_rst'], MIN_RST,
            info['n_rst'] < MIN_RST, 'C7: 只能 SET-only')
    axis_ok = cal[prim].axis_known
    # ★ 该项是【信息项】不是通过项：偏航录制判为"不可辨识"是【正确】行为。
    #   把它当通过项会让"纯偏航数据"被误判为失败，也会掩盖判据写反的 bug。
    add('转轴分量可辨识性(C6, 信息项)', cal[prim].cov_ratio, AXIS_COV_RATIO, None,
        'True 才可用倾角/竖直分量；纯偏航录制应为 False')
    if verbose:
        print('=' * 92)
        print(path)
        print('  帧长 %d  %s  磁样本 %d (SET %d / RST %d)  主相 %s'
              % (fr.step, fr.note, info['n_set'] + info['n_rst'], info['n_set'],
                 info['n_rst'], prim))
        print('  %-32s %14s %10s %6s  %s' % ('验收项', '值', '阈值', '结果', '说明'))
        for n, v, t, ok, note in rows:
            tag = 'INFO' if ok is None else ('PASS' if ok else 'FAIL')
            print('  %-32s %14.4f %10.4f %6s  %s' % (n, v, t, tag, note))
        hard = [r for r in rows if r[3] is not None]
        allp = all(r[3] for r in hard)
        print('  ⟹ %s（%d 项通过项，%d 项信息项）'
              % ('全部通过' if allp else '存在未通过项', len(hard), len(rows) - len(hard)))
        return allp, rows
    hard = [r for r in rows if r[3] is not None]
    return all(r[3] for r in hard), rows


# ============================================================================
# 防错层 —— 本模块的首要目的
# ============================================================================
# 设计原则：不去"提醒"使用者别犯错，而是让错误做法【没有可用入口】。
#   1) 唯一推荐入口是路径 -> Frames -> calibrate -> MagSolver，全程不暴露列号
#   2) 不存在任何"接受零点矢量"的公开 API；零点只能由数据拟合得到
#   3) 相(phase)是强制的：极性随样本走，无法"整批用另一相的球心"
#   4) 几何不支持的量（倾角等）直接抛 GeometryError，不返回"看起来合理"的数
#   5) 单位进名字：mag_counts / accel_g / gyro_dps / *_gauss，不做隐式换算
#   6) 每个阻断项都有 audit_forbidden() 可把"曾经的荒谬结论"当场复现出来
# ----------------------------------------------------------------------------

def midpoint_zero(*_a, **_k):
    """★ 禁用：用 (m_SET+m_RESET)/2 当零点（E4）。

    实测后果：留下 |Δ|/2 = 1794 counts = 场强 21.6% 的体坐标固定偏置，
    使 |B| 出现峰峰 42.1% 的确定性方向调制。曾被误读为
    "MMC 三轴增益不平衡 ±20%"，并进一步被椭球拟合"证实"。
    """
    raise ForbiddenConvention(
        '禁止用两相中点当零点（E4）。实测两相是两个不同心的球，'
        '中点不是任何一相的球心。请改用 calibrate() 得到的分相球心：'
        'SET 与 RST 各自减自己的 center（见 C4）。')


def single_center_fit(*_a, **_k):
    """★ 禁用：把 SET 与 RESET 样本混进同一个球/椭球拟合（E5）。"""
    raise ForbiddenConvention(
        '禁止把两相样本混进同一个拟合（E5）。实测最佳单椭球残差 11.87%，'
        '几何下限 10.76% ⟹ 结构上不可拟合。请用 calibrate()，它按极性自动分相。')


def fixed_zero(*_a, **_k):
    """★ 禁用：跨录制沿用同一个零点常量（E6）。"""
    raise ForbiddenConvention(
        '禁止跨录制复用零点（E6）。实测 SET 相球心跨 6 份录制 x 跨度 38、'
        'y 跨度 204 counts（≈ 水平半径的 5%）。零点必须逐录制解。')


def inclination_deg(cal, u, phase=None):
    """体坐标单位矢量 -> 磁倾角(°)。

    ★ C6 守卫：偏航/单轴录制的球心沿转轴分量不可辨识，
      此时倾角【必然错】且错得看不出异常，所以直接拒绝，不给"看起来合理"的数。
    """
    ph = phase or ('SET' if 'SET' in cal else 'RST')
    c = cal[ph]
    if not c.axis_known:
        raise GeometryError(
            '该录制的样本只覆盖一个圆/平面（cov比 %.3e < %.2f）：球心沿转轴的分量'
            '不可辨识，倾角与竖直分量不可信（C6/E7）。'
            '需用全姿态数据（如六面）标定，或显式确认只使用水平分量。'
            % (c.cov_ratio, AXIS_COV_RATIO))
    u = np.asarray(u, float)
    return float(np.degrees(np.arcsin(np.clip(u[2], -1, 1))))


def unwrap_to_nominal(measured_deg, nominal_deg):
    """把 atan2 出来的角度序列对齐到名义角序列（E12）。

    ★ 必须做这一步：measured 有 ±180 环绕，nominal 可能非单调
      （录里有反向转动），直接做最小二乘会得到 87° 量级的垃圾残差
      —— 这是我自己在本轮犯过两次的错。
    """
    m = np.asarray(measured_deg, float)
    n = np.asarray(nominal_deg, float)
    best = None
    for s in (1.0, -1.0):
        y = m + 360.0 * np.round((s * n - m) / 360.0)
        X = np.column_stack([n, np.ones(len(n))])
        sol, *_ = np.linalg.lstsq(X, y, rcond=None)
        r = y - X @ sol
        if best is None or r.std() < best[0]:
            best = (r.std(), np.abs(r).max(), sol[0], sol[1], s)
    return best          # (残差RMS°, max°, 标度, 截距°, 符号)


def audit_forbidden(fr, verbose=True):
    """把曾经导致"地磁看起来荒谬"的口径当场复现，量化其荒谬程度。

    用途：① 说明本模块为何要这样设计；② 作为回归断言（这些数字必须依然荒谬）。
    """
    newm = fr.new_mask(); idx = np.where(newm)[0]
    p = fr.pol[idx]; M = fr.mag[idx]
    P0, P1 = M[p == 0], M[p == 1]
    out = {}
    cS = fit_phase('SET', P0)
    out['正确口径 |B| 散度%'] = 100 * np.linalg.norm(P0 - cS.center, axis=1).std() \
        / np.linalg.norm(P0 - cS.center, axis=1).mean()
    # E4：中点零点 + 只用 SET
    if len(P1) >= MIN_RST:
        cR = fit_phase('RST', P1)
        cmid = (cS.center + cR.center) / 2.0
        g = np.linalg.norm(P0 - cmid, axis=1)
        out['E4 中点零点 |B| 峰峰%'] = 100 * (np.percentile(g, 99.5) - np.percentile(g, 0.5)) / g.mean()
        out['两相球心距 counts'] = float(np.linalg.norm(cS.center - cR.center))
        out['两相球心距 / |B| %'] = 100 * out['两相球心距 counts'] / cS.radius
        # E5：两相混用单球
        c, R, sd, pct = _sphere(M)
        out['E5 两相混用单球残差%'] = pct
    else:
        out['E4/E5'] = 'RST 样本 < %d，该录制无法分相，只能 SET-only（C7）' % MIN_RST
    # E2：用 DBG 布局读同一批字节（84 B 时才有意义）
    if fr.step == 84:
        wrong = fr.F[:, 14:17]
        mono = bool(np.all(np.diff(wrong[:, 1]) >= 0))
        c2, R2, sd2, pct2 = _sphere(wrong)
        # ★ 不能只报散度：计数器字段能拟出"0% 散度 + 荒谬半径"，看起来反而像很好。
        #   必须把"这不是磁场"这件事直接点出来。
        if mono or not np.isfinite(R2) or R2 > 1e7:
            out['E2 错用 DBG 列'] = ('退化：col15 单调递增（是 imu_cnt 计数器），'
                                     '拟合半径 %.3e counts —— 根本不是磁场' % R2)
        else:
            out['E2 错用 DBG 列后的 |B| 散度%'] = pct2
    # E9：把 84B 原始 counts 当已换算
    if fr.step == 84:
        med = float(np.median(np.linalg.norm(M, axis=1)))
        out['E9 把 counts 当 G 用'] = ('|B| 会报成 %.2f G（真实约 0.51 G，差 %.0f 倍）'
                                       % (med / MAG_CNT_PER_G, (med / MAG_CNT_PER_G) / 0.51))
    if verbose:
        print('  ── audit_forbidden：错误口径的荒谬程度（本模块已阻断）──')
        for k, v in out.items():
            print('     %-34s %s' % (k, ('%.4f' % v) if isinstance(v, float) else v))
    return out


def _sphere(P):
    A = np.column_stack([2 * P[:, 0], 2 * P[:, 1], 2 * P[:, 2], np.ones(len(P))])
    sol, *_ = np.linalg.lstsq(A, (P ** 2).sum(axis=1), rcond=None)
    c = sol[:3]; R = float(np.sqrt(max(sol[3] + c @ c, 0.0)))
    r = np.linalg.norm(P - c, axis=1) - R
    return c, R, float(r.std()), float(100 * r.std() / R)


# ============================================================================
# 自测：合成数据（已知场 + 已知零点 + 已知两相偏置）
# ============================================================================
def selftest(seed=0, verbose=True):
    rng = np.random.default_rng(seed)
    kB = 8318.0                      # 场强 counts（实测六面）
    offS = np.array([129732.7, 131572.0, 131238.6])
    offR = np.array([133038.9, 130480.4, 132106.2])   # 实测两相球心
    n = 20000
    u = rng.normal(size=(n, 3)); u /= np.linalg.norm(u, axis=1)[:, None]  # 全姿态覆盖
    B = u * kB
    mS = offS + B + rng.normal(scale=56.0, size=(n, 3))
    mR = offR - B + rng.normal(scale=58.0, size=(n, 3))
    cal = {'SET': fit_phase('SET', mS), 'RST': fit_phase('RST', mR)}
    ok = []
    ok.append(('SET 残差 <1.5%', cal['SET'].resid_pct < 1.5))
    ok.append(('RST 残差 <1.5%', cal['RST'].resid_pct < 1.5))
    ok.append(('两相均判转轴可辨识', cal['SET'].axis_known and cal['RST'].axis_known))
    # 恢复出的矢量应与真值一致（方向）
    vS = np.array([cal['SET'].to_unit(m) for m in mS[:2000]])
    vR = np.array([cal['RST'].to_unit(m) for m in mR[:2000]])
    # ★ to_unit 对两相都返回【磁场方向】B̂（RST 内部已取反号，见 PhaseCal.to_unit）
    #   ⟹ 两相都必须与同一个 u 比，不能拿 -u 比（这是我第一版的 bug）
    # ★ 阈值不拍常数，而是由【注入噪声】推期望：单样本方向误差中位数
    #   ≈ 1.1774·σ/|B|（三维各向同性高斯噪声下角度的中位数系数）
    sg = {'SET': 56.0, 'RST': 58.0}
    for nm, v in (('SET', vS), ('RST', vR)):
        ref = u[:2000]
        ang = np.degrees(np.arccos(np.clip((v * ref).sum(axis=1), -1, 1)))
        med = float(np.median(ang))
        exp = float(np.degrees(1.1774 * sg[nm] / kB))
        ok.append(('%s 方向误差中位 %.3f° < 1.4×期望 %.3f°' % (nm, med, 1.4 * exp),
                   med < 1.4 * exp))
        sd = 100 * np.linalg.norm(v, axis=1).std()
        ok.append(('%s 模长散度 %.3f%% <1%%' % (nm, sd), sd < 1.0))
    # 两相在同一物理朝向下必须给出一致的磁场方向（跨相一致性）
    cross = np.degrees(np.arccos(np.clip((vS * vR).sum(axis=1), -1, 1)))
    ok.append(('两相跨相一致 中位 %.3f° <1°' % np.median(cross), float(np.median(cross)) < 1.0))
    # 反例：两相混用 + 中点零点必须失败
    cmid = (offS + offR) / 2.0
    gm = np.linalg.norm(np.vstack([mS, mR]) - cmid, axis=1)
    pp = 100 * (np.percentile(gm, 99.5) - np.percentile(gm, 0.5)) / gm.mean()
    ok.append(('反例：中点口径峰峰 >=5%%（实测 %.1f%%）' % pp, pp >= 5.0))
    # 偏航数据必须被判"转轴不可辨识"
    ang_yaw = rng.uniform(0, 2 * np.pi, n)
    axis = np.array([0.005, -0.017, 1.0]); axis /= np.linalg.norm(axis)
    e1 = np.cross(axis, [0, 0, 1.0]); e1 /= np.linalg.norm(e1); e2 = np.cross(axis, e1)
    Byaw = 4040.0 * (np.outer(np.cos(ang_yaw), e1) + np.outer(np.sin(ang_yaw), e2)) + 7300.0 * axis
    cy = fit_phase('SET', offS + Byaw + rng.normal(scale=56.0, size=(n, 3)))
    ok.append(('偏航数据判为转轴不可辨识 (cov比 %.5f)' % cy.cov_ratio, not cy.axis_known))

    # ★★ 防错层必须【真的挡住】，不能只是写在文档里
    for fn, nm in ((midpoint_zero, 'E4 用两相中点当零点'),
                   (single_center_fit, 'E5 两相混进同一拟合'),
                   (fixed_zero, 'E6 跨录制复用零点')):
        try:
            fn(); blocked = False
        except ForbiddenConvention:
            blocked = True
        ok.append(('%s 被阻断' % nm, blocked))
    for cal_t, nm in [({'SET': cy}, '偏航数据求倾角(C6)')]:
        try:
            inclination_deg(cal_t, np.array([0.0, 0.0, 1.0])); blocked = False
        except GeometryError:
            blocked = True
        ok.append(('%s 被拒绝' % nm, blocked))
    # E12：角度对齐助手必须能处理非单调名义序列（否则残差会是 87° 级垃圾）
    nom_test = np.array([0.0, 45, 90, 135, 90, 45, 0, -45.0])
    meas = -nom_test + 7.3
    sd, mx, s, b, sgn = unwrap_to_nominal(meas, nom_test)
    ok.append(('E12 非单调角度对齐 残差RMS %.4f° <0.01' % sd, sd < 0.01))
    if verbose:
        print('=' * 92)
        print('自测（合成数据，含反例）')
        for nm, v in ok:
            print('  %-46s %s' % (nm, 'PASS' if v else 'FAIL'))
        print('  ⟹ %s' % ('全部通过' if all(v for _, v in ok) else '存在未通过项'))
    return all(v for _, v in ok)


if __name__ == '__main__':
    import sys
    ok1 = selftest()
    print()
    files = sys.argv[1:] or [
        r'_baseline_pc_chain_20261012\data\rec_20261012_031605.bin',
        r'R:\rec_20261012_234513.bin',
        r'_recordings\[7]每拍翻转_对照组.bin',
    ]
    import os
    allok = ok1
    for p in files:
        if not os.path.exists(p):
            print('缺文件:', p); continue
        a, _ = accept(p)
        allok = allok and a
        print()
    print('总判定:', '全部通过' if allok else '存在未通过项')
