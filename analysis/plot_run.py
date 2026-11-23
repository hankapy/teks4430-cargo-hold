"""Plot and summarise a run recorded by the recorder node.

Usage: python3 analysis/plot_run.py <log.csv> [more.csv ...]
Writes <log>.png next to each CSV and prints key metrics.
"""

import csv
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

N = 4
CH4_LIMIT = 20.0


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def load_samples(path):
    p = path.replace(".csv", "_samples.csv")
    if not os.path.exists(p):
        return None
    with open(p) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]} if rows else None


def first(t, mask):
    i = np.flatnonzero(mask)
    return float(t[i[0]]) if i.size else None


def metrics(d):
    t = d["t_h"]
    T = np.vstack([d[f"T{i}"] for i in range(N)])
    ch4 = np.vstack([d[f"CH4_{i}"] for i in range(N)]).max(axis=0)
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.0
    hz = d["hotspot_zone"]
    return {
        "duration_h": float(t[-1]),
        "selfheat_confirmed_h": first(t, d["selfheat"] > 0),
        "hotspot_zone": int(hz[-1]),
        "crew_action_h": first(t, d["alert_level"] >= 3),
        "sensor_fault_h": first(t, d["sensor_fault"] > 0),
        "max_T": float(T.max()),
        "max_CH4": float(ch4.max()),
        "h_CH4_over_20": float((ch4 > CH4_LIMIT).sum() * dt),
        "mean_vent": float(d["vent_fraction"].mean()),
        "T60_h": first(t, T.max(axis=0) >= 60.0),
    }


def plot(d, s, title, out):
    t = d["t_h"]
    fig, ax = plt.subplots(5, 1, figsize=(9, 12), sharex=True)
    for i in range(N):
        ax[0].plot(t, d[f"T{i}"], label=f"zone {i}")
        ax[1].plot(t, d[f"CO{i}"], label=f"zone {i}", color=f"C{i}")
        if s is not None:
            m = s["zone"] == i
            ax[1].plot(s["t_h"][m], s["co_ppm"][m], ".", ms=4, color=f"C{i}")
    ax[1].plot(t, d["co_exhaust"], "k:", lw=1, label="exhaust sensor")
    ax[2].plot(t, np.vstack([d[f"CH4_{i}"] for i in range(N)]).max(axis=0), label="max zone (truth)")
    ax[2].plot(t, d["ch4_exhaust"], lw=1, label="exhaust (filtered)")
    ax[2].plot(t, np.where(d["vent_mode"] == 1, d["ch4_setpoint"], np.nan), "g--", lw=1, label="PI setpoint")
    ax[2].axhline(CH4_LIMIT, color="r", lw=0.8, ls="--")
    ax[3].plot(t, d["vent_fraction"], label="vent fraction (measured)")
    ax[4].step(t, d["vent_mode"], where="post", label="vent mode (0 closed, 1 PI, 2 purge)")
    ax[4].step(t, d["alert_level"], where="post", label="alert level")
    ax[4].step(t, d["hotspot_zone"], ":", where="post", label="hotspot zone")
    for a, l in zip(ax, ["hotspot T (C)", "CO (ppm)", "CH4 (% LEL)", "ventilation", "decision"]):
        a.set_ylabel(l)
        a.grid(alpha=0.3)
        a.legend(fontsize=7, loc="upper right")
    ax[0].set_title(title)
    ax[-1].set_xlabel("time since loading (h)")
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    plt.close(fig)


def main():
    for path in sys.argv[1:]:
        d = load(path)
        m = metrics(d)
        plot(d, load_samples(path), os.path.basename(path), path.replace(".csv", ".png"))
        print(os.path.basename(path))
        for k, v in m.items():
            print(f"  {k:22s} {'-' if v is None else (f'{v:.1f}' if isinstance(v, float) else v)}")


if __name__ == "__main__":
    main()
