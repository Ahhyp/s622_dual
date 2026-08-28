#!/usr/bin/env python3
"""
analyze_single_arm.py — 单臂正弦跟踪 / 双臂同步数据分析

指标（docs/2026-08-25_ServoJ动态跟踪与同步性能测试）：
  单臂: RMSE / MAE / max error、幅值比 G=A_actual/A_cmd、相位滞后、等效延迟 τ
  双臂: cross-correlation 相对延迟 + jitter（分窗 mean/std/3σ）

用法：
  # 单臂分析
  python3 analysis/analyze_single_arm.py <csv> --joint left_j1 --freq 0.5 [--warmup 2] [--plot]
  # 双臂同步（csv 含 left_j1+right_j1 两个关节，同频）
  python3 analysis/analyze_single_arm.py <csv> --joint left_j1 --joint right_j1 --freq 0.5 --dual
  # 人工延迟验证（构造 7ms 延迟，cross-correlation 必须恢复 ~7ms）
  python3 analysis/analyze_single_arm.py --delay-check 7.0 --freq 0.5 --fs 500

依赖: numpy（matplotlib 可选，--plot 时需要）
"""
import argparse
import csv
import math
import sys

import numpy as np


def load_csv(path):
    """返回 (cmd_t, cmd_dict, act_t, act_dict)；字典 {joint: ndarray}，时间均相对发送时刻。"""
    cmd_t, act_t = [], []
    cmd_vals, act_vals = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            joints = [k for k in row if k not in ('type', 't_rel', 't_abs')]
            t = float(row['t_rel'])
            vals = {j: float(row[j]) for j in joints}
            if row['type'] == 'cmd':
                cmd_t.append(t); cmd_vals.append(vals)
            else:
                act_t.append(t); act_vals.append(vals)
    cmd_t = np.array(cmd_t)
    act_t = np.array(act_t)
    cmd = {j: np.array([v[j] for v in cmd_vals]) for j in joints}
    act = {j: np.array([v[j] for v in act_vals]) for j in joints}
    return cmd_t, cmd, act_t, act


def align_and_trim(cmd_t, cmd, act_t, act, joint, freq, warmup_cycles):
    """把 actual 插值到 cmd 时间网格；只取 cmd/actual 共同覆盖区间（头尾都要），
    并舍弃 warmup 周期。"""
    period = 1.0 / freq if freq else 0.0
    t_begin = max(warmup_cycles * period, cmd_t[0], act_t[0])
    t_end = min(cmd_t[-1], act_t[-1])
    if t_end <= t_begin:
        raise RuntimeError('command / actual have no common analysis interval')
    mask = (cmd_t >= t_begin) & (cmd_t <= t_end)
    t = cmd_t[mask]
    c = cmd[joint][mask]
    a = np.interp(t, act_t, act[joint])
    return t, c, a


def sine_fit(t, y, freq):
    """最小二乘拟合 y ≈ a*sin(2πft) + b*cos(2πft) + c → (A, phase, offset)。"""
    s = np.sin(2 * np.pi * freq * t)
    cc = np.cos(2 * np.pi * freq * t)
    X = np.column_stack([s, cc, np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    a, b, c = coef
    A = math.hypot(a, b)
    phase = math.atan2(b, a)
    return A, phase, c


def cross_corr_delay(x, y, fs, max_lag_sec=0.1):
    """等间隔序列 x,y 的相对延迟（秒）。约定：返回值 > 0 = y 滞后 x。
    限最大搜索范围 max_lag_sec（正弦周期性会导致整周期假峰，只关心 ±几十 ms）。"""
    n = min(len(x), len(y))
    x = np.asarray(x[:n], dtype=float)
    y = np.asarray(y[:n], dtype=float)
    x -= x.mean()
    y -= y.mean()
    c = np.correlate(x, y, mode='full')
    lags = np.arange(-n + 1, n)
    max_lag_samples = int(round(max_lag_sec * fs))
    valid = np.abs(lags) <= max_lag_samples
    candidate_indices = np.flatnonzero(valid)
    k = candidate_indices[np.argmax(c[valid])]
    # 抛物线插值亚采样精度（把偏移加到 lag 值上，保持 float）
    lag = float(lags[k])
    if 0 < k < len(c) - 1:
        denom = 2.0 * (c[k - 1] - 2.0 * c[k] + c[k + 1])
        if abs(denom) > 1e-12:
            lag += (c[k - 1] - c[k + 1]) / denom
    return -lag / fs


def analyze_single(csv_path, joint, freq, warmup, plot=False):
    cmd_t, cmd, act_t, act = load_csv(csv_path)
    if joint not in cmd:
        print(f"joint '{joint}' not in CSV (have {sorted(cmd)})")
        sys.exit(2)
    t, c, a = align_and_trim(cmd_t, cmd, act_t, act, joint, freq, warmup)
    if len(t) < 10:
        print("too few samples after trimming")
        sys.exit(2)

    e = c - a
    rmse = math.sqrt(float(np.mean(e ** 2)))
    mae = float(np.mean(np.abs(e)))
    emax = float(np.max(np.abs(e)))

    A_cmd, ph_cmd, _ = sine_fit(t, c, freq)
    A_act, ph_act, _ = sine_fit(t, a, freq)
    gain = A_act / A_cmd if A_cmd else float('nan')
    phase_lag = ph_act - ph_cmd            # 负 = actual 滞后
    # 相位差归一化到 [-π, π]
    while phase_lag > math.pi: phase_lag -= 2 * math.pi
    while phase_lag < -math.pi: phase_lag += 2 * math.pi
    tau_ms = -phase_lag / (2 * math.pi * freq) * 1e3   # 等效延迟（正 = 滞后 ms）

    print(f"=== {csv_path}  joint={joint}  f={freq} Hz ===")
    print(f"tracking  RMSE={rmse*1e3:.3f} mrad ({math.degrees(rmse):.4f}°)  "
          f"MAE={mae*1e3:.3f} mrad  max={emax*1e3:.3f} mrad")
    change = (gain - 1.0) * 100.0
    print(f"amplitude A_cmd={math.degrees(A_cmd):.3f}° A_act={math.degrees(A_act):.3f}° "
          f"G={gain:.3f} (change {change:+.1f}%)")
    print(f"phase     lag={math.degrees(phase_lag):.2f}°  "
          f"equivalent delay={tau_ms:.2f} ms")

    if plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
            ax[0].plot(t, c, label='cmd')
            ax[0].plot(t, a, label='actual')
            ax[0].set_ylabel('pos (rad)'); ax[0].legend(); ax[0].grid()
            ax[1].plot(t, e * 1e3)
            ax[1].set_xlabel('t (s)'); ax[1].set_ylabel('error (mrad)'); ax[1].grid()
            out = csv_path.rsplit('.', 1)[0] + '_plot.png'
            fig.savefig(out)
            print(f"plot saved: {out}")
        except Exception as ex:
            print(f"(plot skipped: {ex})")

    return dict(rmse=rmse, mae=mae, emax=emax, gain=gain, delay_ms=tau_ms)


def load_csv_dual(path):
    """读取 CSV 的 actual 段，使用相对统一 T0 的 t_rel（左右文件共享同一 T0）。"""
    t, rows = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            if row['type'] != 'actual':
                continue
            t.append(float(row['t_rel']))
            rows.append({j: float(row[j]) for j in row if j not in ('type', 't_rel', 't_abs')})
    if not rows:
        raise RuntimeError(f'no actual records in {path}')
    t = np.asarray(t)
    joints = sorted(rows[0])
    return t, {j: np.asarray([r[j] for r in rows]) for j in joints}


def get_cmd_t0(path):
    """返回 CSV 第一条 cmd 的 t_abs（=统一 T0）。"""
    with open(path) as f:
        for row in csv.DictReader(f):
            if row['type'] == 'cmd':
                return float(row['t_abs'])
    raise RuntimeError(f'no cmd records in {path}')


def analyze_dual_files(csvL, csvR, jointL, jointR, freq, warmup, window_sec):
    """双臂同步：同一次同步实验（一个 sine_tracking_test 进程发左右两个 controller，
    共享同一 T0）产生的左右 CSV。相对延迟 + 分窗 jitter（约定 >0 = right 滞后 left）。
    交叉验证：cross-correlation 延迟 与 正弦相位差延迟。"""
    # 两个 CSV 必须来自同一 T0（防止拿不同实验的数据算同步）
    t0L, t0R = get_cmd_t0(csvL), get_cmd_t0(csvR)
    if abs(t0L - t0R) > 1e-6:
        raise RuntimeError(
            f'CSV files do not share the same T0: L={t0L:.9f}, R={t0R:.9f} — '
            f'not from the same synchronized experiment')
    tL, dL = load_csv_dual(csvL)
    tR, dR = load_csv_dual(csvR)
    if jointL not in dL or jointR not in dR:
        print(f"joints not found: have L={sorted(dL)} R={sorted(dR)}")
        sys.exit(2)
    # 统一网格 + warmup（t_rel 相对 T0，warmup 才真正生效）
    t_begin = max(tL[0], tR[0], warmup / freq)
    t_end = min(tL[-1], tR[-1])
    if t_end <= t_begin:
        raise RuntimeError('left/right have no common analysis interval')
    t = np.arange(t_begin, t_end, 0.005)
    aL = np.interp(t, tL, dL[jointL])
    aR = np.interp(t, tR, dR[jointR])
    fs = 1.0 / np.median(np.diff(t))

    # 方法 1：cross-correlation（全段）
    delay_all = cross_corr_delay(aL, aR, fs) * 1e3
    # 方法 2：正弦相位差（交叉验证）
    _, phL, _ = sine_fit(t, aL, freq)
    _, phR, _ = sine_fit(t, aR, freq)
    dphi = phR - phL
    dphi = (dphi + math.pi) % (2 * math.pi) - math.pi
    phase_delay_ms = -dphi / (2 * math.pi * freq) * 1e3

    # 分窗 jitter（t 已从 warmup 后的 t_begin 开始）
    n_per_win = max(2, int(window_sec * fs))
    delays = []
    for i in range(0, len(t) - n_per_win, n_per_win):
        delays.append(cross_corr_delay(aL[i:i + n_per_win], aR[i:i + n_per_win], fs) * 1e3)
    delays = np.array(delays)
    if len(delays) < 5:
        raise RuntimeError(f'too few jitter windows: {len(delays)}')
    mu, sigma = float(np.mean(delays)), float(np.std(delays, ddof=1))
    print(f"=== dual sync (two CSV)  {jointL} vs {jointR}  f={freq} Hz ===")
    print(f"  {csvL}\n  {csvR}   (shared T0 = {t0L:.3f} s)")
    print(f"cross-corr delay (R lags L, +) : {delay_all:+.2f} ms  (full record)")
    print(f"phase-fit delay                : {phase_delay_ms:+.2f} ms  (cross-validation)")
    print(f"windowed ({window_sec}s): mean={mu:+.2f} ms  σ={sigma:.2f} ms  "
          f"3σ={3*sigma:.2f} ms  n={len(delays)}")
    return dict(delay_ms=delay_all, phase_delay_ms=phase_delay_ms,
                mean_ms=mu, sigma_ms=sigma)


def analyze_dual(csv_path, joints, freq, warmup, window_sec):
    """双臂同步：两个关节（left_j1/right_j1）执行相同轨迹，算相对延迟 + 分窗 jitter。"""
    cmd_t, cmd, act_t, act = load_csv(csv_path)
    jL, jR = joints[0], joints[1]
    period = 1.0 / freq
    t0 = warmup * period
    mask = cmd_t >= t0
    t = cmd_t[mask]
    # 插值到统一网格（cmd 网格）
    fs = 1.0 / np.median(np.diff(t))
    aL = np.interp(t, act_t, act[jL])
    aR = np.interp(t, act_t, act[jR])

    # 整段相对延迟
    delay_all = cross_corr_delay(aL, aR, fs) * 1e3

    # 分窗 jitter
    n_per_win = max(2, int(window_sec * fs))
    delays = []
    for i in range(0, len(t) - n_per_win, n_per_win):
        d = cross_corr_delay(aL[i:i + n_per_win], aR[i:i + n_per_win], fs) * 1e3
        delays.append(d)
    delays = np.array(delays)
    mu, sigma = float(np.mean(delays)), float(np.std(delays))
    print(f"=== dual sync {csv_path}  {jL} vs {jR}  f={freq} Hz ===")
    print(f"relative delay (R lags L, +) : {delay_all:+.2f} ms  (full record)")
    print(f"windowed ({window_sec}s): mean={mu:+.2f} ms  σ={sigma:.2f} ms  3σ={3*sigma:.2f} ms  "
          f"n={len(delays)}")
    return dict(delay_ms=delay_all, mean_ms=mu, sigma_ms=sigma)


def delay_check(delay_ms, freq, fs, dur=60.0):
    """人工注入已知延迟，验证 cross-correlation 恢复精度。"""
    t = np.arange(0, dur, 1.0 / fs)
    x = np.sin(2 * np.pi * freq * t)
    y = np.sin(2 * np.pi * freq * (t - delay_ms / 1e3))
    est = cross_corr_delay(x, y, fs) * 1e3
    err = abs(est - delay_ms)
    print(f"delay-check: injected={delay_ms:+6.2f} ms  estimated={est:+6.3f} ms  "
          f"err={err:.3f} ms  {'OK' if err < 0.05 else 'FAIL'}")
    return est


def delay_regression(freq=0.5, fs=500.0, tol_ms=0.05):
    """固定回归测试：+7 / -7 / +20 ms 三组，误差超阈值即失败。"""
    print(f"=== delay regression (f={freq} Hz, fs={fs} Hz, tol={tol_ms} ms) ===")
    ok = True
    for d in (7.0, -7.0, 20.0):
        est = delay_check(d, freq, fs)
        if abs(est - d) > tol_ms:
            ok = False
    print("regression:", "PASS" if ok else "FAIL")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?', help='CSV from sine_tracking_test.py')
    ap.add_argument('--joint', action='append', default=[], help='joint(s), repeatable')
    ap.add_argument('--freq', type=float, default=0.5)
    ap.add_argument('--warmup', type=int, default=2, help='discard first N cycles')
    ap.add_argument('--plot', action='store_true')
    ap.add_argument('--dual', action='store_true', help='dual-arm sync analysis (single CSV)')
    ap.add_argument('--dual-files', nargs=2, metavar=('CSV_L', 'CSV_R'),
                    help='dual-arm sync: two CSV from separate left/right instances')
    ap.add_argument('--jointL', default='left_j1')
    ap.add_argument('--jointR', default='right_j1')
    ap.add_argument('--window', type=float, default=2.0, help='jitter window (s)')
    ap.add_argument('--delay-check', type=float, metavar='MS', help='synthetic delay validation')
    ap.add_argument('--regression', action='store_true',
                    help='run fixed delay regression (+7/-7/+20 ms)')
    ap.add_argument('--fs', type=float, default=500.0, help='sample rate for delay-check')
    args = ap.parse_args()

    if args.regression:
        delay_regression(args.freq, args.fs)
        return
    if args.delay_check is not None:
        delay_check(args.delay_check, args.freq, args.fs)
        return

    if not args.csv and not args.dual_files:
        ap.error('csv or --dual-files required')
    if args.dual_files:
        analyze_dual_files(args.dual_files[0], args.dual_files[1],
                           args.jointL, args.jointR, args.freq, args.warmup, args.window)
        return
    if args.dual:
        if len(args.joint) < 2:
            ap.error('--dual needs --joint left_j1 --joint right_j1')
        analyze_dual(args.csv, args.joint, args.freq, args.warmup, args.window)
    else:
        joint = args.joint[0] if args.joint else None
        if not joint:
            # 自动选 CSV 里第一个关节
            _, cmd, _, _ = load_csv(args.csv)
            joint = sorted(cmd)[0]
            print(f"(auto-selected joint {joint})")
        analyze_single(args.csv, joint, args.freq, args.warmup, args.plot)


if __name__ == '__main__':
    main()
