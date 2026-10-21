"""Checks a simulated run (run_timing_simulation.py) against ground truth.

Reports, separately:
  1. EXPERIMENTAL timing: durations of INFLATING/HOLDING/DEFLATING in the
     recorded EEG state (samples), per condition, and their error vs nominal.
  2. PHYSICAL timing: when the controller THINKS the cuff reached the target /
     emptied (Arduino event mapped to the PC clock) vs when the firmware
     really emitted the event (simulator truth), plus where physical events
     fall relative to the EEG event boundaries.
  3. PRESSURE: final pressure error vs target (true cuff pressure), overshoot,
     and LOW/NONE ramp tracking.

    python analyze_simulation.py <sim_out_dir> [--plot fig.png]
"""

import csv
import json
import os
import sys
from collections import defaultdict

import numpy as np

LAB = {0: "NONE", 1: "LOW", 2: "HIGH"}
PH = {1: "INFLATING", 2: "HOLDING", 3: "DEFLATING"}


def load(d):
    eeg = json.load(open(os.path.join(d, "eeg_states.json")))
    res = json.load(open(os.path.join(d, "sim_results.json")))
    T, E = [], []
    for row in open(os.path.join(d, "truth.csv")):
        p = row.rstrip("\r\n").split(",", 2)
        if p[0] == "T":
            a = row.split(",")
            T.append((float(a[1]), float(a[2]), int(a[3]), int(a[4])))
        elif p[0] == "E":
            E.append((float(p[1]), p[2].strip()))
    return eeg, res, np.array(T), E


def eeg_segments(eeg):
    fs, bs = eeg["fs"], eeg["block"]
    blocks = eeg["blocks"]
    ph = np.array([b[2]["StimulusPhase"] for b in blocks])
    tn = np.array([b[2]["TrialNumber"] for b in blocks])
    pl = np.array([b[2]["PainLevel"] for b in blocks])
    tsend = np.array([b[1] for b in blocks])
    ch = np.flatnonzero(np.diff(ph) != 0) + 1
    starts = np.r_[0, ch]
    ends = np.r_[ch, len(ph)]
    segs = []
    for s, e in zip(starts, ends):
        segs.append(dict(trial=int(tn[s]), pain=int(pl[s]), phase=int(ph[s]),
                         start_block=int(s), n_blocks=int(e - s),
                         samples=int((e - s) * bs), start_send_time=float(tsend[s])))
    return segs, fs, bs, tsend


def main():
    d = sys.argv[1]
    plot = sys.argv[sys.argv.index("--plot") + 1] if "--plot" in sys.argv else None
    eeg, res, T, E = load(d)
    segs, fs, bs, tsend = eeg_segments(eeg)
    block_s = bs / fs
    cal = res["calibration"]
    mode = res["sync"]["sync_mode"]
    nominal = {1: cal["inflating_samples"], 2: cal["holding_samples"], 3: cal["deflating_samples"]}

    print(f"=== {os.path.basename(os.path.normpath(d))}: sync mode {mode}, "
          f"{fs:g} Hz, block {bs} ({block_s*1000:.1f} ms) ===")
    print(f"calibration: HIGH inflation {cal['MEASURED_HIGH_INFLATION_TIME']:.3f} s -> "
          f"INFLATING {cal['INFLATING_DURATION']} s ({nominal[1]} samples); "
          f"HIGH deflation {cal['MEASURED_HIGH_DEFLATION_TIME']:.3f} s -> "
          f"DEFLATING {cal['DEFLATING_DURATION']} s ({nominal[3]} samples); "
          f"HOLDING {nominal[2]} samples")

    # ---------------- 1. experimental timing (EEG) ----------------
    trial_segs = defaultdict(dict)
    for s in segs:
        if s["trial"] >= 1 and s["phase"] in (0, 1, 2, 3):
            trial_segs[s["trial"]][s["phase"]] = s
    print("\n1) EXPERIMENTAL timing in the recorded state (samples)")
    dur = defaultdict(lambda: defaultdict(list))
    for t, ss in sorted(trial_segs.items()):
        for p in (1, 2, 3):
            if p in ss:
                dur[p][LAB[ss[p]["pain"]]].append(ss[p]["samples"])
            else:
                dur[p]["MISSING"].append(t)
    all_ok = True
    for p in (1, 2, 3):
        vals = {c: sorted(set(v)) for c, v in dur[p].items()}
        allv = [x for c, v in dur[p].items() if c != "MISSING" for x in v]
        err_ms = (np.array(allv) - nominal[p]) / fs * 1000.0
        same = len(set(allv)) == 1 and "MISSING" not in dur[p]
        all_ok &= same and allv[0] == nominal[p]
        print(f"   {PH[p]:9s} nominal {nominal[p]:5d} | " +
              " | ".join(f"{c}: {v} (n={len(dur[p][c])})" for c, v in sorted(vals.items())) +
              f" | error vs nominal: max |{np.abs(err_ms).max():.1f}| ms"
              f" | identical across conditions: {'YES' if same else 'NO'}")
    fix = [ss[0]["samples"] for ss in trial_segs.values() if 0 in ss]
    print(f"   (FIXATION: {sorted(set(fix))} samples; not a standardized event)")

    # send error (Python side)
    se = defaultdict(list)
    for tr in res["trials"]:
        for k, v in tr["timing"]["experimental_s"].items():
            if v.get("send_error_ms") is not None:
                se[k].append(v["send_error_ms"])
    print("   Python send time - scheduled deadline (ms): " + ", ".join(
        f"{k} mean {np.mean(v):.2f} max {np.max(v):.2f}" for k, v in se.items()))

    # landing: where the EEG onset falls relative to the command
    # (EEG onset of a block = acquisition start = send_time - block_s)
    land = defaultdict(list)
    for tr in res["trials"]:
        tim = tr["timing"]
        t0 = tim["t0_perf_counter"]
        ss = trial_segs.get(tr["trial"], {})
        for p in (1, 2, 3):
            name = PH[p]
            if p in ss and name in tim["experimental_s"]:
                cmd = t0 + tim["experimental_s"][name]["sent_s"]
                onset = ss[p]["start_send_time"] - block_s
                land[name].append((onset - cmd) * 1000.0)
    print("   EEG sample-onset minus command time (ms)  [sets the physical<->EEG offset]:")
    for k, v in land.items():
        print(f"      {k:9s} mean {np.mean(v):7.1f}  sd {np.std(v):5.1f}  "
              f"range {np.min(v):7.1f} .. {np.max(v):7.1f}")

    # ---------------- 2. physical timing ----------------
    print("\n2) PHYSICAL timing")
    # truth: firmware emission time of each event line
    ev_truth = {}
    for t, line in E:
        p = line.split(",")
        if p[0] in ("PUMP_ON", "TARGET_REACHED", "DEFLATING", "DEFLATED", "INFLATING") and len(p) >= 3:
            ev_truth.setdefault((int(p[1]), p[0]), t)
    keymap = {"pump_on": "PUMP_ON", "target_reached": "TARGET_REACHED",
              "deflate_valve_open": "DEFLATING", "deflated": "DEFLATED"}
    map_err = defaultdict(list)
    rx_err = defaultdict(list)
    phys_rel_eeg = defaultdict(lambda: defaultdict(list))
    rows = []
    for tr in res["trials"]:
        tim = tr["timing"]
        t0 = tim["t0_perf_counter"]
        ss = trial_segs.get(tr["trial"], {})
        eeg_inf = ss[1]["start_send_time"] - block_s if 1 in ss else None
        eeg_hold = ss[2]["start_send_time"] - block_s if 2 in ss else None
        eeg_def = ss[3]["start_send_time"] - block_s if 3 in ss else None
        eeg_end = eeg_def + ss[3]["n_blocks"] * block_s if 3 in ss else None
        for k, w in keymap.items():
            if k in tim["physical_s"] and (tr["trial"], w) in ev_truth:
                truth = ev_truth[(tr["trial"], w)]
                map_err[k].append((t0 + tim["physical_s"][k] - truth) * 1000.0)
                rx_err[k].append((t0 + tim["physical_rx_s"][k] - truth) * 1000.0)
        tr_truth = ev_truth.get((tr["trial"], "TARGET_REACHED"))
        df_truth = ev_truth.get((tr["trial"], "DEFLATED"))
        c = tr["condition"]
        if tr_truth and eeg_inf:
            phys_rel_eeg["target_reached - EEG INFLATING onset"][c].append(tr_truth - eeg_inf)
            phys_rel_eeg["EEG HOLDING onset - target_reached"][c].append(eeg_hold - tr_truth)
        if df_truth and eeg_def:
            phys_rel_eeg["deflated - EEG DEFLATING onset"][c].append(df_truth - eeg_def)
            phys_rel_eeg["EEG DEFLATING end - deflated"][c].append(eeg_end - df_truth)
        rows.append((tr, eeg_inf))
    print("   Controller's estimate of Arduino event time minus true firmware time (ms):")
    for k in keymap:
        if map_err[k]:
            v, r = np.array(map_err[k]), np.array(rx_err[k])
            print(f"      {k:19s} clock-mapped: mean {v.mean():6.2f} sd {v.std():5.2f} "
                  f"max|{np.abs(v).max():5.2f}|   | raw arrival time: mean {r.mean():6.2f} max {r.max():6.2f}")
    print("   Physical events relative to EEG boundaries (s; truth):")
    for k, byc in phys_rel_eeg.items():
        print(f"      {k:38s} " + " | ".join(
            f"{c}: {np.min(v):.3f}..{np.max(v):.3f}" for c, v in sorted(byc.items())))
    early_hold = sum(1 for tr in res["trials"]
                     if tr["timing"].get("target_reached_before_inflating_end") is False)
    print(f"   Trials where the target was reached AFTER INFLATING ended: {early_hold}")
    print("   -> physical completion never changed the state: every duration above is fixed.")

    # ---------------- 3. pressure ----------------
    print("\n3) PRESSURE (true cuff pressure from the simulator)")
    tt, pp = T[:, 0], T[:, 1]
    perf = defaultdict(lambda: defaultdict(list))
    for tr, eeg_inf in rows:
        tim = tr["timing"]
        t0 = tim["t0_perf_counter"]
        tgt = tr["pressure"]
        c = tr["condition"]
        hold_start = t0 + tim["experimental_s"]["HOLDING"]["sent_s"]
        def_start = t0 + tim["experimental_s"]["DEFLATING"]["sent_s"]
        m_inf = (tt >= t0) & (tt < def_start)
        perf["max true - target (overshoot)"][c].append(pp[m_inf].max() - tgt)
        i_h = np.searchsorted(tt, hold_start)
        perf["true at HOLDING onset - target"][c].append(pp[i_h] - tgt)
        i_d = np.searchsorted(tt, def_start) - 1
        perf["true at DEFLATING onset - target"][c].append(pp[i_d] - tgt)
        if "target_reached" in tim["physical_pressure_mmhg"]:
            perf["sensor reading at TARGET_REACHED - target"][c].append(
                tim["physical_pressure_mmhg"]["target_reached"] - tgt)
        rt = tim.get("ramp_tracking_error_mmhg")
        if rt:
            perf["ramp: sensed - reference, worst below"][c].append(rt["min"])
            perf["ramp: sensed - reference, worst above"][c].append(rt["max"])
        if "target_reached" in tim["physical_s"]:
            perf["target reached at (s after INFLATING)"][c].append(tim["physical_s"]["target_reached"])
        if "deflated" in tim["physical_s"]:
            perf["physical deflation (s)"][c].append(tim["physical_deflation_s"])
    for k, byc in perf.items():
        print(f"   {k:44s} " + " | ".join(
            f"{c}: {np.mean(v):+.2f} [{np.min(v):+.2f}, {np.max(v):+.2f}]" for c, v in sorted(byc.items())))

    print(f"\nArduino clock error estimated by the controller: {res.get('arduino_clock_ppm')} ppm "
          f"(simulated: {res['args']['clock_ppm']:+.0f} ppm)")
    print(f"block-align failures: {res.get('align_failures')}")
    print(f"\nRESULT: all standardized event durations identical and equal to nominal: "
          f"{'PASS' if all_ok else 'FAIL'}")

    if plot:
        make_plot(plot, rows, T, trial_segs, block_s, cal, fs)


def make_plot(path, rows, T, trial_segs, block_s, cal, fs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    color = {"HIGH": "#2a78d6", "LOW": "#eb6834", "NONE": "#1baf7a"}
    ink, muted, grid = "#1f1f1e", "#6b6a64", "#e6e5df"
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4), sharey=False)
    tt, pp = T[:, 0], T[:, 1]
    d_inf = cal["inflating_scheduled_s"]
    d_hold = cal["holding_scheduled_s"]
    d_def = cal["deflating_scheduled_s"]
    for ax, c in zip(axes, ("HIGH", "LOW", "NONE")):
        for x0, x1, lab in ((0, d_inf, "INFLATING"), (d_inf, d_inf + d_hold, "HOLDING"),
                            (d_inf + d_hold, d_inf + d_hold + d_def, "DEFLATING")):
            ax.axvspan(x0, x1, color=grid, alpha=0.55 if lab != "HOLDING" else 0.25, lw=0)
            ax.text((x0 + x1) / 2, 1.01, lab, transform=ax.get_xaxis_transform(),
                    ha="center", va="bottom", fontsize=8, color=muted)
        tgt = None
        for tr, eeg_inf in rows:
            if tr["condition"] != c or eeg_inf is None:
                continue
            tgt = tr["pressure"]
            m = (tt >= eeg_inf - 1) & (tt <= eeg_inf + d_inf + d_hold + d_def + 1)
            ax.plot(tt[m] - eeg_inf, pp[m], color=color[c], lw=1.4, alpha=0.8)
            t_tr = tr["timing"]["physical_s"].get("target_reached")
            if t_tr is not None:
                t0 = tr["timing"]["t0_perf_counter"]
                ax.plot([t0 + t_tr - eeg_inf], [tgt], "o", ms=5, color=color[c],
                        mec="white", mew=1.2)
        if tgt:
            ax.axhline(tgt, color=muted, lw=0.8, ls="--")
            if c != "HIGH":
                ramp_end = cal["low_ramp_ms"] / 1000.0
                ax.plot([0, ramp_end], [0, tgt], color=ink, lw=0.9, ls=":",
                        label="ramp reference")
                ax.legend(loc="upper right", frameon=False, fontsize=8)
        ax.set_title(f"{c}  ({tgt} mmHg)", loc="left", fontsize=11, color=ink, pad=18)
        ax.set_xlabel("s from EEG INFLATING onset", color=muted, fontsize=9)
        ax.set_xlim(-1, d_inf + d_hold + d_def + 1)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("left", "bottom"):
            ax.spines[s].set_color(grid)
        ax.tick_params(colors=muted, labelsize=8)
    axes[0].set_ylabel("true cuff pressure (mmHg)", color=muted, fontsize=9)
    fig.suptitle("Simulated trials: physical pressure inside fixed experimental windows "
                 "(dots = TARGET_REACHED as logged)", fontsize=11, color=ink, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print("plot ->", path)


if __name__ == "__main__":
    main()
