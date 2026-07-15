#!/usr/bin/env python3
"""
track_perf.py — Reproducible tracking-time recorder for VINS-Mono / SPA-Lite
============================================================================

Designed for the EuRoC + sparse-alignment comparison, but works for any
ros2 launch that prints these lines on stdout/stderr:

    [feature_tracker_node-1] [INFO] [...]: [TRACKER_STATS] frames=N mean_total=Xms
    [feature_tracker_node-1] [INFO] [...]: [KLT_STATS]     frames=N mean_cost=Xms win=W lvls=L sparse_prior=P%
    [feature_tracker_node-1] [INFO] [...]: [SPARSE_STATS]  frames=A/B success_rate=X% mean_nmeas=.. mean_chi2=.. mean_time=Xms mean_klt=..

NOTE: feature_tracker.cpp uses RCUTILS_LOG_INFO, which writes to stderr
but does NOT publish to /rosout. So we cannot subscribe to /rosout to
capture these lines — we MUST read the launch process's own
stdout/stderr. launch.ros2 prefixes each line with [<node_name>-<idx>]
which we strip before regex-matching.

Two modes:

  (a) DRIVE  (recommended)
      The script launches `ros2 bag play` and your launch command
      itself, tails their merged stdout, and writes the parsed STATS
      into a per-run directory. This is the "5-run mean" workflow used
      in docs/article/BENCHMARK_SUMMARY.txt.

        python3 track_perf.py --seq MH03 --mode sparse_off \
          --bag ~/bags/MH_03_medium.db3 --rate 1.0 \
          --runs 5 --bin-start 1500 --bin-count 3 \
          --launch-cmd "ros2 launch vins_estimator vins_launch.py" \
          --out output/MH03

  (b) TAIL   (manual launch)
      You launch the pipeline yourself and pipe its stdout into a
      FIFO; track_perf tails that FIFO. Use this if you need to keep
      full control of the bag/launch lifecycle.

        mkfifo /tmp/track_perf.fifo
        ros2 launch vins_estimator vins_launch.py 2>&1 > /tmp/track_perf.fifo &
        ros2 bag play ~/bags/MH_03_medium.db3
        python3 track_perf.py --tail /tmp/track_perf.fifo \
          --seq MH03 --mode sparse_off --tag manual \
          --bin-start 1500 --bin-count 3 --out output/MH03
        # Ctrl-C when done

Outputs per-run directory `output/<seq>/<mode>_<tag>/`:

    tracking_time.csv               # every parsed TRACKER/KLT/SPARSE bin
    summary.json                    # final bin mean values + bin range
    launch.log                      # full captured stdout (DRIVE/TAIL)
    bag.log                         # full bag-play stdout (DRIVE only)
    cmdline.txt                     # exact command line, for reproducibility
    <mode>_<tag>_aggregate.json     # DRIVE mode only — N-run mean ± std
"""

import argparse
import csv
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path


# ---------------------------------------------------------------------------
# Regexes — must match the printf format inside feature_tracker.cpp
# ---------------------------------------------------------------------------

RE_TRACKER = re.compile(
    r"\[TRACKER_STATS\]\s+frames=(?P<frames>\d+)\s+mean_total=(?P<mean_total>[\d.]+)ms"
)
RE_KLT = re.compile(
    r"\[KLT_STATS\]\s+frames=(?P<frames>\d+)\s+mean_cost=(?P<klt>[\d.]+)ms\s+"
    r"mean_win=(?P<win>[\d.]+)\s+mean_lvls=(?P<lvls>[\d.]+)\s+"
    r"sparse_prior=(?P<sparse_prior>[\d.]+)%"
)
RE_SPARSE = re.compile(
    r"\[SPARSE_STATS\]\s+frames=(?P<a>\d+)/(?P<b>\d+)\s+"
    r"success_rate=(?P<success>[\d.]+)%\s+"
    r"mean_nmeas=(?P<nmeas>\d+)\s+"
    r"mean_chi2=(?P<chi2>[\d.]+)\s+"
    r"mean_time=(?P<time>[\d.]+)ms\s+"
    r"mean_klt=(?P<klt>[\d.]+)"
)

# launch.ros2 prefixes every node stdout line as "[<node>-<idx>]".
# Strip it before matching — but only for STATS lines, since we still
# want to keep it in launch.log for debugging.
# Multiple nesting levels exist: [node] [INFO] [ts] []: [STATS]
RE_LAUNCH_PREFIX = re.compile(r"^(\[[^\]]*\]\s*)+")


def now_iso() -> str:
    return datetime.now().strftime("%Y-%m-%dT%H:%M:%S")


def parse_and_emit(line, writer, csv_fd):
    """Strip launch prefix, match STATS, append CSV rows."""
    bare = RE_LAUNCH_PREFIX.sub("", line).strip()
    ts = now_iso()
    emitted = False
    m = RE_TRACKER.search(bare)
    if m:
        writer.writerow([ts, "TRACKER", int(m["frames"]),
                         float(m["mean_total"]), "", "", "", "", "", "", "", ""])
        emitted = True
    m = RE_KLT.search(bare)
    if m:
        writer.writerow([ts, "KLT", int(m["frames"]),
                         "", float(m["klt"]), "",
                         float(m["win"]), float(m["lvls"]),
                         float(m["sparse_prior"]), "", "", ""])
        emitted = True
    m = RE_SPARSE.search(bare)
    if m:
        writer.writerow([ts, "SPARSE", int(m["b"]),
                         "", "", float(m["time"]), "", "", "",
                         int(m["nmeas"]), float(m["chi2"]),
                         float(m["success"])])
        emitted = True
    if emitted:
        csv_fd.flush()
    return emitted


def summarize_csv(csv_path, bin_start, bin_count, extra=None):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    trackers = [r for r in rows
                if r["kind"] == "TRACKER" and int(r["frames"]) >= bin_start]
    trackers = trackers[-bin_count:] if trackers else []
    klts = [r for r in rows
            if r["kind"] == "KLT" and int(r["frames"]) >= bin_start]
    klts = klts[-bin_count:] if klts else []
    sparses = [r for r in rows
               if r["kind"] == "SPARSE" and int(r["frames"]) >= bin_start]
    sparses = sparses[-bin_count:] if sparses else []

    def avg(rows, key):
        vals = [float(r[key]) for r in rows if r[key]]
        return sum(vals) / len(vals) if vals else None

    summary = {
        "n_tracker_bins": len(trackers),
        "n_klt_bins": len(klts),
        "n_sparse_bins": len(sparses),
        "bin_start_frames": bin_start,
        "bin_count": bin_count,
        "mean_total_ms": avg(trackers, "mean_total_ms"),
        "klt_ms": avg(klts, "klt_ms"),
        "spa_ms": avg(sparses, "spa_ms"),
        "klt_win": avg(klts, "klt_win"),
        "klt_lvls": avg(klts, "klt_lvls"),
        "sparse_prior_pct": avg(klts, "sparse_prior_pct"),
        "spa_success_pct": avg(sparses, "spa_success_pct"),
        "spa_nmeas": (sum(int(r["spa_nmeas"]) for r in sparses) / len(sparses)
                      if sparses else None),
        "spa_chi2": avg(sparses, "spa_chi2"),
        "recorded_at": now_iso(),
    }
    if extra:
        summary.update(extra)
    return summary


def open_csv(out_dir):
    csv_path = out_dir / "tracking_time.csv"
    fd = open(csv_path, "w", newline="")
    writer = csv.writer(fd)
    writer.writerow(
        ["timestamp", "kind", "frames",
         "mean_total_ms", "klt_ms", "spa_ms",
         "klt_win", "klt_lvls", "sparse_prior_pct",
         "spa_nmeas", "spa_chi2", "spa_success_pct"]
    )
    fd.flush()
    return csv_path, fd, writer


# ---------------------------------------------------------------------------
# DRIVE mode — spawn bag + launch, tail launch stdout
# ---------------------------------------------------------------------------

def drive_run(args, run_idx):
    tag = f"{args.mode}_{args.tag}_r{run_idx+1:02d}" if args.tag \
          else f"{args.mode}_r{run_idx+1:02d}"
    out_dir = Path(args.out) / args.seq / tag
    out_dir.mkdir(parents=True, exist_ok=True)

    with open(out_dir / "cmdline.txt", "w") as f:
        f.write(" ".join(sys.argv) + "\n")

    bag_log = open(out_dir / "bag.log", "w")
    bag_proc = subprocess.Popen(
        ["ros2", "bag", "play", args.bag, "--rate", str(args.rate)]
        + (["--start", str(args.bag_start)] if args.bag_start else []),
        stdout=bag_log, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    launch_cmd = args.launch_cmd.split() if args.launch_cmd else [
        "ros2", "launch", "vins_estimator", "vins_launch.py",
        f"enable_sparse:=1" if args.mode == "sparse_on" else "enable_sparse:=0"
    ]
    launch_log = open(out_dir / "launch.log", "w")
    launch_proc = subprocess.Popen(
        launch_cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        bufsize=1, text=True,
        preexec_fn=os.setsid,
    )

    csv_path, csv_fd, writer = open_csv(out_dir)

    bin_width = args.bin_width
    record_secs = max(args.warmup_secs + (args.bin_start / 20.0)
                      + args.bin_count * bin_width, 30.0)

    start = time.time()
    stats_lines = {"TRACKER": 0, "KLT": 0, "SPARSE": 0}

    try:
        while True:
            elapsed = time.time() - start
            if elapsed > record_secs:
                break
            if launch_proc.poll() is not None:
                tail = launch_proc.stdout.read() or ""
                launch_log.write(tail)
                for ln in tail.splitlines():
                    if parse_and_emit(ln, writer, csv_fd):
                        # We don't know which kind was hit — recompute
                        # by re-running the regex; cheaper to just
                        # track totals via the parser return.
                        pass
                launch_log.flush()
                break
            line = launch_proc.stdout.readline()
            if not line:
                time.sleep(0.05)
                continue
            launch_log.write(line)
            launch_log.flush()
            m_t = RE_TRACKER.search(RE_LAUNCH_PREFIX.sub("", line))
            m_k = RE_KLT.search(RE_LAUNCH_PREFIX.sub("", line))
            m_s = RE_SPARSE.search(RE_LAUNCH_PREFIX.sub("", line))
            if parse_and_emit(line, writer, csv_fd):
                if m_t: stats_lines["TRACKER"] += 1
                if m_k: stats_lines["KLT"] += 1
                if m_s: stats_lines["SPARSE"] += 1
    finally:
        csv_fd.close()

    for proc in (launch_proc, bag_proc):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
    for proc in (launch_proc, bag_proc):
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    bag_log.close()
    launch_log.close()

    summary = summarize_csv(
        csv_path, args.bin_start, args.bin_count,
        extra={"run_idx": run_idx, "tag": tag,
               "record_secs": record_secs,
               "stats_lines": stats_lines},
    )
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    return summary, out_dir


def aggregate_runs(run_summaries, args):
    keys = ["mean_total_ms", "klt_ms", "spa_ms", "klt_win", "klt_lvls",
            "sparse_prior_pct", "spa_success_pct"]
    agg = {}
    for k in keys:
        vals = [s[k] for s in run_summaries if s.get(k) is not None]
        agg[f"{k}_mean"] = (sum(vals) / len(vals)) if vals else None
        agg[f"{k}_std"] = (
            (sum((v - agg[f"{k}_mean"]) ** 2 for v in vals) / len(vals)) ** 0.5
            if vals and len(vals) > 1 else None
        )
        agg[f"{k}_n"] = len(vals)
    agg["runs"] = args.runs
    agg["recorded_at"] = now_iso()
    return agg


# ---------------------------------------------------------------------------
# PARALLEL mode — run sparse_on and sparse_off simultaneously
# ---------------------------------------------------------------------------

def parallel_run(args):
    """Run sparse_on and sparse_off in parallel with the same bag.
    Starts ONE vins_compare_launch.py (both pipelines), TWO bag plays,
    and merges log output into per-mode CSVs by namespace filtering."""
    out_dir = Path(args.out) / args.seq / f"parallel_{now_iso().replace(':', '-')}"
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "cmdline.txt", "w") as f:
        f.write(" ".join(sys.argv) + "\n")

    # Compute record time
    bin_width = args.bin_width
    record_secs = max(args.warmup_secs + (args.bin_start / 20.0)
                      + args.bin_count * bin_width, 30.0)

    # Shared launch output pipe
    launch_cmd = ["ros2", "launch", "vins_estimator", "vins_compare_launch.py"]
    launch_proc = subprocess.Popen(
        launch_cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        bufsize=1, text=True,
        preexec_fn=os.setsid,
    )

    csv_paths = {}
    for mode in ("sparse_off", "sparse_on"):
        mode_dir = out_dir / mode
        mode_dir.mkdir(parents=True, exist_ok=True)
        csv_paths[mode] = mode_dir / "tracking_time.csv"

    # Single bag play (both pipelines subscribe to same topics)
    bag_args = ["ros2", "bag", "play", args.bag, "--rate", str(args.rate)]
    if args.bag_start:
        bag_args += ["--start", str(args.bag_start)]
    bag_log = open(out_dir / "bag.log", "w")
    bag_proc = subprocess.Popen(
        bag_args,
        stdout=bag_log, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    # Single log watcher, merge into both CSVs by namespace
    stop_event = threading.Event()
    stats = {"sparse_off": {"TRACKER": 0, "KLT": 0, "SPARSE": 0},
             "sparse_on":  {"TRACKER": 0, "KLT": 0, "SPARSE": 0}}
    stats_lock = threading.Lock()

    csv_fds = {mode: open(csv_paths[mode], "w", newline="") for mode in csv_paths}
    writers = {mode: csv.writer(csv_fds[mode]) for mode in csv_paths}
    for mode in csv_paths:
        writers[mode].writerow(
            ["timestamp", "kind", "frames",
             "mean_total_ms", "klt_ms", "spa_ms",
             "klt_win", "klt_lvls", "sparse_prior_pct",
             "spa_nmeas", "spa_chi2", "spa_success_pct"]
        )
        csv_fds[mode].flush()

    ns_filters = {mode: re.compile(rf"\/{mode}\/") for mode in ("sparse_off", "sparse_on")}

    try:
        while True:
            line = launch_proc.stdout.readline()
            if not line:
                time.sleep(0.01)
                continue
            # Write to both launch logs
            for mode in ("sparse_off", "sparse_on"):
                with open(out_dir / mode / "launch.log", "a") as lf:
                    lf.write(line)
                    lf.flush()
            # Parse and emit to matching mode CSVs
            for mode in ("sparse_off", "sparse_on"):
                if not ns_filters[mode].search(line):
                    continue
                bare = RE_LAUNCH_PREFIX.sub("", line).strip()
                if parse_and_emit(line, writers[mode], csv_fds[mode]):
                    with stats_lock:
                        if RE_TRACKER.search(bare): stats[mode]["TRACKER"] += 1
                        if RE_KLT.search(bare):     stats[mode]["KLT"] += 1
                        if RE_SPARSE.search(bare):  stats[mode]["SPARSE"] += 1
    except KeyboardInterrupt:
        pass
    finally:
        for fd in csv_fds.values():
            fd.close()
        stop_event.set()

    # Kill all procs
    try:
        os.killpg(os.getpgid(bag_proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    time.sleep(2)
    try:
        bag_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(bag_proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    bag_log.close()
    try:
        launch_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass

    # Summarize
    results = {}
    for mode in ("sparse_off", "sparse_on"):
        summary = summarize_csv(
            csv_paths[mode], args.bin_start, args.bin_count,
            extra={"mode": mode, "stats_lines": stats[mode].copy(),
                   "record_secs": record_secs},
        )
        with open(out_dir / mode / "summary.json", "w") as f:
            json.dump(summary, f, indent=2)
        results[mode] = summary

    # Write comparison
    comparison = {
        "sparse_off": results["sparse_off"],
        "sparse_on": results["sparse_on"],
        "diff": {},
        "recorded_at": now_iso(),
        "out_dir": str(out_dir),
    }
    for k in ["mean_total_ms", "klt_ms", "spa_ms"]:
        off = results["sparse_off"].get(k)
        on = results["sparse_on"].get(k)
        if off is not None and on is not None:
            comparison["diff"][k] = round(on - off, 4)

    with open(out_dir / "comparison.json", "w") as f:
        json.dump(comparison, f, indent=2)

    return results, out_dir


# ---------------------------------------------------------------------------
# TAIL mode — read a FIFO until EOF or Ctrl-C
# ---------------------------------------------------------------------------

def tail_run(args):
    tag = f"{args.mode}_{args.tag or 'manual'}"
    out_dir = Path(args.out) / args.seq / tag
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "cmdline.txt", "w") as f:
        f.write(" ".join(sys.argv) + "\n")

    csv_path, csv_fd, writer = open_csv(out_dir)
    launch_log = open(out_dir / "launch.log", "w")
    stats_lines = {"TRACKER": 0, "KLT": 0, "SPARSE": 0}

    print(f"track_perf: tailing {args.tail} -> {out_dir}  (Ctrl-C to stop)")
    try:
        with open(args.tail, "r") as f:
            for line in f:
                launch_log.write(line)
                launch_log.flush()
                m_t = RE_TRACKER.search(RE_LAUNCH_PREFIX.sub("", line))
                m_k = RE_KLT.search(RE_LAUNCH_PREFIX.sub("", line))
                m_s = RE_SPARSE.search(RE_LAUNCH_PREFIX.sub("", line))
                if parse_and_emit(line, writer, csv_fd):
                    if m_t: stats_lines["TRACKER"] += 1
                    if m_k: stats_lines["KLT"] += 1
                    if m_s: stats_lines["SPARSE"] += 1
    except KeyboardInterrupt:
        pass
    finally:
        csv_fd.close()
        launch_log.close()

    summary = summarize_csv(
        csv_path, args.bin_start, args.bin_count,
        extra={"tag": tag, "stats_lines": stats_lines,
               "tail_source": args.tail},
    )
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    return summary, out_dir


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def resolve_out_root(args):
    out = Path(args.out).resolve()
    if out.name == args.seq and out.parent != Path(".").resolve():
        return out.parent
    return out


def parse_args():
    p = argparse.ArgumentParser(
        description="Reproducible tracking-time recorder for VINS-Mono / SPA-Lite",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--seq", required=True, help="Sequence tag, e.g. MH02")
    p.add_argument("--mode", required=True,
                   choices=["sparse_on", "sparse_off", "both"],
                   help="sparse_on=sparse ON, sparse_off=sparse OFF, both=run both simultaneously")
    p.add_argument("--tag", default="", help="Run tag, e.g. v1 (optional)")
    p.add_argument("--out", default="output",
                   help="Output root directory; per-run subdirs are created beneath it")
    p.add_argument("--bin-start", type=int, default=1500,
                   help="Skip tracker bins before this frame count (TRACKER_STATS uses %% 50)")
    p.add_argument("--bin-count", type=int, default=3,
                   help="Number of trailing bins to average")
    p.add_argument("--bin-width", type=float, default=2.5,
                   help="Approx seconds between TRACKER_STATS lines (50 frames / 20 Hz)")

    # DRIVE-mode options
    p.add_argument("--bag", default="", help="Ros2 bag to play (DRIVE mode)")
    p.add_argument("--bag-start", type=float, default=0.0,
                   help="Bag --start seconds offset")
    p.add_argument("--rate", type=float, default=1.0, help="Bag play rate")
    p.add_argument("--runs", type=int, default=1,
                   help="Number of repeated runs in DRIVE mode")
    p.add_argument("--launch-cmd", default="",
                   help="Custom launch command, e.g. "
                        "'ros2 launch vins_estimator vins_launch.py enable_sparse:=1'")
    p.add_argument("--warmup-secs", type=float, default=5.0)

    # TAIL-mode option (mutually exclusive with --bag in practice)
    p.add_argument("--tail", default="",
                   help="Path to FIFO/file to tail (TAIL mode). "
                        "When set, --bag / --runs are ignored.")

    return p.parse_args()


def main():
    args = parse_args()
    out_root = resolve_out_root(args)
    args.out = str(out_root)

    if args.tail:
        # TAIL mode
        summary, out_dir = tail_run(args)
        print("\n=== summary ===")
        print(json.dumps(summary, indent=2))
        print(f"\nSaved -> {out_dir}/summary.json")
        return

    if not args.bag:
        sys.stderr.write(
            "ERROR: pass either --tail <fifo> (TAIL mode) or "
            "--bag <bag.db3> (DRIVE mode). See --help.\n"
        )
        sys.exit(2)

    if args.mode == "both":
        # PARALLEL mode: run both sparse_on and sparse_off simultaneously
        results, out_dir = parallel_run(args)
        print("\n=== comparison ===")
        print(f"out_dir: {out_dir}")
        for mode, summary in results.items():
            print(f"\n--- {mode} ---")
            print(json.dumps(summary, indent=2))
        return

    # DRIVE mode (sparse_on or sparse_off)
    run_summaries = []
    for i in range(args.runs):
        print(f"\n[{i+1}/{args.runs}] starting run ...")
        s, od = drive_run(args, i)
        run_summaries.append(s)
        print(f"  -> {od}/summary.json")
        if i + 1 < args.runs:
            time.sleep(2.0)

    agg = aggregate_runs(run_summaries, args)
    agg_name = f"{args.mode}_{args.tag}_aggregate.json" if args.tag \
               else f"{args.mode}_aggregate.json"
    agg_path = Path(args.out) / args.seq / agg_name
    with open(agg_path, "w") as f:
        json.dump(agg, f, indent=2)
    print("\n=== aggregate ===")
    print(json.dumps(agg, indent=2))
    print(f"\nSaved -> {agg_path}")


if __name__ == "__main__":
    main()