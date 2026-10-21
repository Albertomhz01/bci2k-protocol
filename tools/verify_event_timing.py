"""Checks the experimental event timing inside real BCI2000 .dat recordings.

For every trial it measures, in EEG samples, how long StimulusPhase stayed at
INFLATING (1), HOLDING (2) and DEFLATING (3), grouped by PainLevel
(0 NONE, 1 LOW, 2 HIGH), and reports whether the durations are identical
across conditions. Optionally compares with the calibrated durations stored
in the results JSON of the same run.

Needs only numpy (no BCI2000 installation).

    python tools/verify_event_timing.py dat_bci/P001/P001S001R06.dat
    python tools/verify_event_timing.py run.dat --results participant_001_run06_results.json
"""

import argparse
import json
import re
from collections import defaultdict

import numpy as np

LAB = {0: "NONE", 1: "LOW", 2: "HIGH"}
PH = {0: "FIXATION", 1: "INFLATING", 2: "HOLDING", 3: "DEFLATING", 4: "REST"}


def read_dat(path):
    raw = open(path, "rb").read()
    first = raw[:raw.index(b"\n")].decode("latin1")
    hl = int(re.search(r"HeaderLen=\s*(\d+)", first).group(1))
    nch = int(re.search(r"SourceCh=\s*(\d+)", first).group(1))
    svl = int(re.search(r"StatevectorLen=\s*(\d+)", first).group(1))
    fmt = re.search(r"DataFormat=\s*(\w+)", first)
    size = {"int16": 2, "int32": 4, "float32": 4}[fmt.group(1) if fmt else "int16"]
    hdr = raw[:hl].decode("latin1").replace("\r", "")
    states, params, sec = {}, {}, None
    for line in hdr.split("\n"):
        if line.startswith("["):
            sec = line
            continue
        if sec and "State Vector" in sec and line.strip():
            p = line.split()
            states[p[0]] = (int(p[1]), int(p[3]), int(p[4]))
        m = re.search(r" (\w+)= (\S+)", line)
        if sec and "Parameter" in sec and m:
            params[m.group(1)] = m.group(2)
    rec = nch * size + svl
    body = np.frombuffer(raw[hl:], dtype=np.uint8)
    n = len(body) // rec
    sv = body[:n * rec].reshape(n, rec)[:, nch * size:].astype(np.uint64)

    def state(name):
        length, byte, bit = states[name]
        val = np.zeros(n, dtype=np.uint64)
        for b in range((bit + length + 7) // 8):
            if byte + b < svl:
                val |= sv[:, byte + b] << np.uint64(8 * b)
        return ((val >> np.uint64(bit)) & np.uint64((1 << length) - 1)).astype(np.int64)

    fs = float(re.match(r"[\d.]+", params["SamplingRate"]).group(0))
    bs = int(re.match(r"\d+", params["SampleBlockSize"]).group(0))
    return fs, bs, n, state


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dat")
    ap.add_argument("--results", help="participant_XXX_runYY_results.json of the same run")
    a = ap.parse_args()

    fs, bs, n, st = read_dat(a.dat)
    ph, tn, pl = st("StimulusPhase"), st("TrialNumber"), st("PainLevel")
    print(f"{a.dat}: {n} samples, {fs:g} Hz, SampleBlockSize {bs} "
          f"({bs / fs * 1000:.1f} ms per block, state changes are quantized to it)")

    ch = np.flatnonzero(np.diff(ph) != 0) + 1
    print(f"state changes on a block boundary: {np.mean(ch % bs == 0) * 100:.0f} %")
    starts, ends = np.r_[0, ch], np.r_[ch, n]

    seq = defaultdict(list)
    for s, e in zip(starts, ends):
        seq[(int(tn[s]), int(pl[s]))].append((int(ph[s]), int(e - s)))

    nominal = {}
    if a.results:
        r = json.load(open(a.results))
        cal = r.get("calibration", {})
        for p, key in ((1, "inflating_samples"), (2, "holding_samples"), (3, "deflating_samples")):
            if cal.get(key):
                nominal[p] = cal[key]
        if nominal:
            print(f"calibrated durations (samples): " +
                  ", ".join(f"{PH[p]} {v}" for p, v in nominal.items()))

    dur = defaultdict(lambda: defaultdict(list))
    missing = defaultdict(list)
    for (trial, pain), segs in sorted(seq.items()):
        if trial < 1:
            continue
        phases = [p for p, _ in segs]
        for p in (1, 2, 3):
            if p not in phases:
                missing[p].append((trial, LAB.get(pain, pain)))
        for p, d in segs:
            if p in (1, 2, 3):
                dur[p][LAB.get(pain, pain)].append(d)

    ok = True
    for p in (1, 2, 3):
        allv = [x for v in dur[p].values() for x in v]
        same = len(set(allv)) == 1 and not missing[p]
        if p in nominal:
            same = same and allv and allv[0] == nominal[p]
        ok &= same
        print(f"\n{PH[p]}: {'IDENTICAL' if same else 'NOT identical'} across trials/conditions")
        for c, v in sorted(dur[p].items()):
            u, k = np.unique(v, return_counts=True)
            print(f"   {c:5s} n={len(v):2d}  " + ", ".join(
                f"{x} samples ({x / fs:.3f} s) x{cnt}" for x, cnt in zip(u, k)))
        if missing[p]:
            print(f"   MISSING in {len(missing[p])} trials (never reached the EEG): {missing[p]}")
    print(f"\nRESULT: {'PASS' if ok else 'FAIL'}")


if __name__ == "__main__":
    main()
