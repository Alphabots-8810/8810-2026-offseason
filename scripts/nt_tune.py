"""
nt_tune.py — live NetworkTables (NT4) PID monitor.

Connects to the robot's NetworkTables server as an NT4 client, subscribes to the
AdvantageKit telemetry tree, and reports the same closed-loop tracking quality
that pid_analysis.py computes from .wpilog files — but LIVE, with no USB stick.

This is READ-ONLY: it never writes to NetworkTables, so it cannot change robot
behavior. It only listens.

Usage:
    python scripts/nt_tune.py                       # simulation (127.0.0.1)
    python scripts/nt_tune.py --host 172.22.11.2    # roboRIO over USB tether
    python scripts/nt_tune.py --team 8810           # roboRIO over radio/ethernet
    python scripts/nt_tune.py --window 6 --interval 3   # 6s rolling window, report every 3s

Where to connect:
    simulation       127.0.0.1
    USB tether       172.22.11.2
    radio/ethernet   10.88.10.2   (or just --team 8810)

Requires robotpy-ntcore (the NT4 client). Python 3.15 betas have no wheels yet,
so use a 3.13 venv:
    py -3.13 -m venv .venv-tune
    .venv-tune\\Scripts\\activate
    pip install robotpy-ntcore
"""

import argparse
import collections
import sys
import threading
import time
from pathlib import Path

# Reuse the per-pair error reporting from the offline log parser. (Its discover_pairs
# targets Phoenix ClosedLoopReference naming; this robot's MotorIO layer uses a simpler
# "<name>Setpoint<unit>" convention, so pairing is done locally below.)
sys.path.insert(0, str(Path(__file__).parent))
from pid_analysis import report_pair, suggest  # noqa: E402

try:
    import ntcore
except ImportError:
    sys.exit(
        "ntcore not found. Install the NT4 client into a Python 3.13 venv:\n"
        "    py -3.13 -m venv .venv-tune\n"
        "    .venv-tune\\Scripts\\activate\n"
        "    pip install robotpy-ntcore"
    )

# AdvantageKit publishes its whole tree to NT under this prefix. The offline log
# stores the same signals without it (e.g. /RealOutputs/...), and pid_analysis's
# patterns expect that bare form, so we strip the prefix when building records.
AKIT_PREFIX = "/AdvantageKit"


def _timestamp_seconds(value):
    """Best-effort sample time in seconds, preferring the robot (server) clock."""
    for getter in ("server_time", "time"):
        fn = getattr(value, getter, None)
        if fn is not None:
            t = fn()
            if t:  # skip 0 (clock not yet synced)
                return t / 1e6
    return time.monotonic()


class LiveRecords:
    """Thread-safe rolling buffer of name -> deque[(t_seconds, value)]."""

    def __init__(self, window_seconds):
        self.window = window_seconds
        self.lock = threading.Lock()
        self.data = collections.defaultdict(collections.deque)

    def add(self, name, t, v):
        with self.lock:
            self.data[name].append((t, v))

    def snapshot(self):
        """Return a plain dict (sorted lists) trimmed to the rolling window."""
        with self.lock:
            now = max(
                (dq[-1][0] for dq in self.data.values() if dq),
                default=0.0,
            )
            cutoff = now - self.window
            out = {}
            for name, dq in self.data.items():
                while dq and dq[0][0] < cutoff:
                    dq.popleft()
                if dq:
                    out[name] = sorted(dq)
            return out


def _describe(measurement_key):
    """Derive (subsystem, quantity, unit) from a measurement key's leaf name."""
    parts = measurement_key.strip("/").split("/")
    subsystem = parts[0] if parts else measurement_key
    leaf = parts[-1]
    for suffix, quantity, unit in (
        ("RadPerSec", "velocity", "rad/s"),
        ("RotPerSec", "velocity", "rot/s"),
        ("DegPerSec", "velocity", "deg/s"),
        ("Rad", "position", "rad"),
        ("Rot", "position", "rot"),
        ("Deg", "position", "deg"),
    ):
        if leaf.endswith(suffix):
            return subsystem, quantity, unit
    return subsystem, "value", ""


def discover_pairs_live(records, include_idle=False):
    """
    Pair each setpoint signal with its measurement using the MotorIO convention:
    a key containing 'Setpoint' maps to the same key with 'Setpoint' removed.

    By default, loops whose setpoint never leaves ~0 in the window are skipped so the
    report surfaces only the loop currently being commanded. include_idle keeps them.
    """
    pairs = []
    for key in records:
        if "Setpoint" not in key:
            continue
        measurement = key.replace("Setpoint", "")
        if measurement not in records:
            continue
        if not include_idle:
            sp_vals = [abs(v) for _, v in records[key]]
            if not sp_vals or max(sp_vals) < 1e-6:
                continue  # loop idle — not being commanded
        subsystem, quantity, unit = _describe(measurement)
        pairs.append((measurement, key, f"{subsystem} {quantity}", unit))
    return sorted(pairs)


def make_listener(records):
    def on_event(event):
        data = getattr(event, "data", None)
        if data is None:
            return
        topic = getattr(data, "topic", None)
        value = getattr(data, "value", None)
        if topic is None or value is None:
            return
        v = value.value()
        if isinstance(v, bool):
            v = float(v)
        elif isinstance(v, (int, float)):
            v = float(v)
        else:
            return  # only numeric closed-loop signals matter here
        name = topic.getName()
        if name.startswith(AKIT_PREFIX):
            name = name[len(AKIT_PREFIX):]
        records.add(name, _timestamp_seconds(value), v)

    return on_event


def main():
    # The reused report uses box-drawing / block glyphs; force UTF-8 so they don't
    # crash on Windows' default GBK console codec.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass

    ap = argparse.ArgumentParser(description="Live NT4 PID tracking monitor (read-only).")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--host", help="NT server host (default 127.0.0.1 for sim).")
    g.add_argument("--team", type=int, help="Team number (connects to the roboRIO).")
    ap.add_argument("--window", type=float, default=8.0, help="Rolling analysis window, seconds.")
    ap.add_argument("--interval", type=float, default=2.0, help="Seconds between reports.")
    ap.add_argument(
        "--all",
        action="store_true",
        help="Report every discovered loop, including idle ones (default: active only).",
    )
    args = ap.parse_args()

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("claude-nt-tuner")
    if args.team:
        inst.setServerTeam(args.team)
        target = f"team {args.team}"
    else:
        host = args.host or "127.0.0.1"
        inst.setServer(host, ntcore.NetworkTableInstance.kDefaultPort4)
        target = host

    records = LiveRecords(args.window)
    sub = ntcore.MultiSubscriber(inst, [AKIT_PREFIX + "/"])
    inst.addListener(sub, ntcore.EventFlags.kValueAll, make_listener(records))

    print(f"Connecting to {target} (NT4)…  (Ctrl-C to stop)")
    for _ in range(100):  # up to ~5s
        if inst.isConnected():
            break
        time.sleep(0.05)
    if not inst.isConnected():
        print("  Still not connected. Is the robot/sim running and is this PC on its network?")
        print("  Continuing to retry in the background…")

    try:
        while True:
            time.sleep(args.interval)
            records_snapshot = records.snapshot()
            print(f"\n{'='*62}")
            status = "connected" if inst.isConnected() else "DISCONNECTED"
            print(f"  Live PID tracking — {target} [{status}]  window={args.window:.0f}s")
            print(f"  Numeric signals seen: {len(records_snapshot)}")
            print(f"{'='*62}")

            if not inst.isConnected():
                continue

            pairs = discover_pairs_live(records_snapshot, include_idle=args.all)
            if not pairs:
                hint = (
                    "  No setpoint/measurement pairs found."
                    if args.all
                    else "  No actively-commanded loops. Run a subsystem, or pass --all to see idle loops."
                )
                print(hint)
                continue

            for actual_key, sp_key, label, unit in pairs:
                print()
                vel_peak = None
                if "deg/s" in unit:
                    vels = [v for _, v in records_snapshot.get(actual_key, [])]
                    vel_peak = max(abs(min(vels, default=0)), abs(max(vels, default=0)))
                ss = report_pair(actual_key, sp_key, label, unit, records_snapshot)
                suggest(label, unit, ss, vel_peak)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        inst.stopClient()


if __name__ == "__main__":
    main()
