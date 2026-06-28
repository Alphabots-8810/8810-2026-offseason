"""
pid_analysis.py — reads a .wpilog (AdvantageKit) and reports PID tracking
quality for every closed-loop subsystem found in the log.

Usage:
    python scripts/pid_analysis.py                        # auto-picks newest akit log on F:\
    python scripts/pid_analysis.py path/to/file.wpilog    # specific file
    python scripts/pid_analysis.py F:/logs/               # scan a directory

Outputs per subsystem:
  - setpoint vs actual error stats (mean bias, RMS, peak, steady-state)
  - plain-English tuning suggestion
"""

import struct, sys, collections, math, re
from pathlib import Path

# ── WPILOG binary parser ──────────────────────────────────────────────────────

def _read_var(data, pos, size):
    return int.from_bytes(data[pos:pos+size], 'little'), pos + size

def parse_wpilog(path):
    """Return dict: signal_name -> list of (timestamp_seconds, value)."""
    data = Path(path).read_bytes()
    if data[:6] != b'WPILOG':
        raise ValueError(f"{path} is not a WPILOG file")
    extra_len = struct.unpack_from('<I', data, 8)[0]
    pos = 12 + extra_len
    entries = {}
    records = collections.defaultdict(list)

    while pos < len(data):
        if pos >= len(data): break
        bit_field = data[pos]; pos += 1
        id_size  = (bit_field & 0x03) + 1
        pay_size = ((bit_field >> 2) & 0x03) + 1
        ts_size  = ((bit_field >> 4) & 0x07) + 1
        if pos + id_size + pay_size + ts_size > len(data): break
        entry_id, pos = _read_var(data, pos, id_size)
        payload_len, pos = _read_var(data, pos, pay_size)
        timestamp, pos = _read_var(data, pos, ts_size)
        if pos + payload_len > len(data): break
        payload = data[pos:pos+payload_len]; pos += payload_len

        if entry_id == 0:                          # control record
            if not payload: continue
            if payload[0] == 0 and len(payload) >= 9:   # start record
                p = 1
                eid = struct.unpack_from('<I', payload, p)[0]; p += 4
                nlen = struct.unpack_from('<I', payload, p)[0]; p += 4
                name = payload[p:p+nlen].decode('utf-8', errors='replace'); p += nlen
                tlen = struct.unpack_from('<I', payload, p)[0]; p += 4
                typ  = payload[p:p+tlen].decode('utf-8', errors='replace')
                entries[eid] = (name, typ)
        else:
            if entry_id not in entries: continue
            name, typ = entries[entry_id]
            try:
                if   typ == 'double': val = struct.unpack_from('<d', payload)[0]
                elif typ == 'float':  val = struct.unpack_from('<f', payload)[0]
                elif typ == 'int64':  val = struct.unpack_from('<q', payload)[0]
                elif typ == 'boolean': val = float(payload[0])
                else: continue
                records[name].append((timestamp / 1e6, val))
            except struct.error:
                pass
    return records

# ── utilities ─────────────────────────────────────────────────────────────────

def interp(series, t):
    """Linearly interpolate a (timestamp, value) series at time t."""
    if not series: return None
    if t <= series[0][0]: return series[0][1]
    if t >= series[-1][0]: return series[-1][1]
    lo, hi = 0, len(series) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t: lo = mid
        else: hi = mid
    t0, v0 = series[lo]; t1, v1 = series[hi]
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0)

def _stat(vals):
    if not vals: return None
    n = len(vals)
    mean = sum(vals) / n
    rms  = math.sqrt(sum(v*v for v in vals) / n)
    return {'n': n, 'mean': mean, 'rms': rms, 'min': min(vals), 'max': max(vals)}

def pair_errors(actual_series, setpoint_series):
    """Return list of (setpoint - actual) errors sampled at actual timestamps."""
    errors = []
    for t, actual in actual_series:
        sp = interp(setpoint_series, t)
        if sp is not None:
            errors.append(sp - actual)
    return errors

def steady_state(errors, tail_fraction=0.20):
    """RMS of the last tail_fraction of the error series (≈ steady state)."""
    tail = errors[int(len(errors) * (1 - tail_fraction)):]
    s = _stat([abs(e) for e in tail])
    return s['rms'] if s else None

# ── auto-discovery of closed-loop subsystem pairs ─────────────────────────────

# Patterns that pair (actual, setpoint) signals.
# Each entry: (actual_pattern, setpoint_pattern, label, unit)
PAIR_PATTERNS = [
    # Motion Magic position
    (r'(.+)/(\w+)PositionRot$',
     r'\1/\2ClosedLoopReferenceRot',
     'MM position', 'rot'),
    # Motion Magic velocity
    (r'(.+)/(\w+)VelocityRotPerSec$',
     r'\1/\2ClosedLoopReferenceSlopeRotPerSec',
     'MM velocity', 'rot/s'),
    # Generic velocity in RPS logged under RealOutputs setpoint
    (r'/Shooter/Flywheel(\d+)LeaderVelocityRotationPerSec$',
     r'/RealOutputs/Shooter/FlywheelSetpointRPS',
     'Flywheel{1} velocity', 'RPS'),
    # Hood: actual position (deg) vs setpoint (deg)
    (r'/RealOutputs/(.+)/SetpointDeg$',
     r'/RealOutputs/\1/SetpointDeg',    # placeholder — hood has no actual pos logged
     'Hood position', 'deg'),
]

def discover_pairs(records):
    """
    Auto-discover (actual, setpoint) pairs present in the log.
    Returns list of (actual_key, setpoint_key, label, unit).
    """
    keys = list(records.keys())
    found = []
    seen_actual = set()

    # Motion Magic: look for any ClosedLoopReference → find the matching actual
    for key in keys:
        if 'ClosedLoopReferenceRot' in key and 'Slope' not in key:
            # key = /Subsystem/FooClosedLoopReferenceRot
            actual_key = key.replace('ClosedLoopReferenceRot', 'PositionRot')
            if actual_key in records and actual_key not in seen_actual:
                label = key.split('/')[1] if key.count('/') >= 2 else key
                found.append((actual_key, key, f'{label} position (MM)', 'rot'))
                seen_actual.add(actual_key)

        if 'ClosedLoopReferenceSlopeRotPerSec' in key:
            actual_key = key.replace('ClosedLoopReferenceSlopeRotPerSec', 'VelocityRotPerSec')
            if actual_key in records and actual_key not in seen_actual:
                label = key.split('/')[1] if key.count('/') >= 2 else key
                found.append((actual_key, key, f'{label} velocity (MM)', 'rot/s'))
                seen_actual.add(actual_key)

    # Flywheel: match each leader to the generic setpoint
    sp_rps = '/RealOutputs/Shooter/FlywheelSetpointRPS'
    for key in keys:
        if re.search(r'Flywheel\d+LeaderVelocity', key) and sp_rps in records:
            m = re.search(r'(Flywheel\d+)', key)
            label = m.group(1) if m else 'Flywheel'
            found.append((key, sp_rps, f'{label} velocity', 'RPS'))

    # Hood: SetpointDeg + VelocityDegPerSec (no direct position logged)
    for key in keys:
        if key.endswith('VelocityDegPerSec') and not key.startswith('/RealOutputs'):
            subsys = key.split('/')[1]
            sp_key = f'/RealOutputs/{subsys}/SetpointDeg'
            if sp_key in records:
                found.append((key, sp_key, f'{subsys} (velocity-only check)', 'deg/s'))

    return found

# ── reporting ─────────────────────────────────────────────────────────────────

def _bar(val, max_val=1.0, width=20):
    n = int(min(val / max_val, 1.0) * width)
    return '█' * n + '░' * (width - n)

def report_pair(actual_key, setpoint_key, label, unit, records):
    actual  = records.get(actual_key, [])
    setpnt  = records.get(setpoint_key, [])

    if len(actual) < 5 or len(setpnt) < 5:
        print(f"  {label}: not enough samples (actual={len(actual)}, setpoint={len(setpnt)})")
        return None

    errors = pair_errors(actual, setpnt)
    if not errors:
        print(f"  {label}: no overlapping timestamps")
        return None

    abs_e = [abs(e) for e in errors]
    s  = _stat(errors)
    sa = _stat(abs_e)
    ss = steady_state(errors)

    print(f"  ┌─ {label}  [{unit}]")
    print(f"  │  samples        : {s['n']}")
    print(f"  │  mean bias      : {s['mean']:+.4f}  {'(lagging)' if s['mean'] > 0 else '(leading)'}")
    print(f"  │  error RMS      : {sa['rms']:.4f}")
    print(f"  │  peak error     : {sa['max']:.4f}")
    print(f"  │  steady-state   : {ss:.4f}  {_bar(ss, sa['max'] if sa['max'] else 1)}")
    return ss

def suggest(label, unit, ss_error, vel_peak=None):
    """Print a plain-English tuning suggestion."""
    print(f"  └─ Suggestion:")
    if 'position' in label.lower() or 'MM' in label:
        if ss_error is not None:
            if ss_error > 0.05:
                print(f"       Steady-state error {ss_error:.4f} {unit} is HIGH.")
                print(f"       → Increase kP (try doubling: 0.1 → 0.2)")
                print(f"       → Add kS ≈ 0.3–0.6 A to overcome static friction")
            elif ss_error > 0.01:
                print(f"       Error {ss_error:.4f} {unit} is moderate.")
                print(f"       → Add kS ≈ 0.2–0.4 A; increase kP slightly")
            else:
                print(f"       Error {ss_error:.4f} {unit} — tracking looks GOOD.")
                print(f"       → Optionally add kS to reduce lag at motion start")
    elif 'velocity' in label.lower() or 'flywheel' in label.lower():
        if ss_error is not None:
            pct = ss_error  # already in unit
            print(f"       Steady-state velocity error: {ss_error:.3f} {unit}")
            if ss_error > 2.0:
                print(f"       → Add kV feedforward (≈ 1/free_speed in {unit})")
                print(f"       → Add kS ≈ 0.2–0.5 A for friction")
                print(f"       → kP alone cannot hold speed against disturbances")
            else:
                print(f"       → Velocity tracking reasonable; add kV to reduce spin-up time")
    if vel_peak is not None and vel_peak > 80:
        print(f"       Peak velocity {vel_peak:.1f} {unit} is high for range → add kD to damp oscillation")

# ── log selection ─────────────────────────────────────────────────────────────

def pick_log(arg=None):
    if arg:
        p = Path(arg)
        if p.is_file():
            return [p]
        if p.is_dir():
            logs = sorted(p.glob('akit_*.wpilog'))
            return [logs[-1]] if logs else []
    # auto: newest akit log on F drive
    for drive in ['F', 'E', 'D']:
        d = Path(f'{drive}:/logs')
        if d.exists():
            logs = sorted(d.glob('akit_*.wpilog'))
            if logs:
                return [logs[-1]]
    return []

# ── main ─────────────────────────────────────────────────────────────────────

def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    logs = pick_log(arg)
    if not logs:
        print("No .wpilog found. Pass a file or directory path as argument.")
        sys.exit(1)

    for log_path in logs:
        print(f"\n{'='*62}")
        print(f"  Log: {log_path.name}")
        size_mb = log_path.stat().st_size / 1e6
        print(f"  Size: {size_mb:.1f} MB")
        print(f"{'='*62}")

        records = parse_wpilog(log_path)
        print(f"  Signals parsed: {len(records)}\n")

        pairs = discover_pairs(records)
        if not pairs:
            print("  No closed-loop pairs found in this log.")
            print("  (Was the robot enabled and running closed-loop control?)")
            continue

        for actual_key, sp_key, label, unit in pairs:
            print()
            # velocity peak for hood oscillation check
            vel_peak = None
            if 'deg/s' in unit:
                vels = [v for _, v in records.get(actual_key, [])]
                vel_peak = max(abs(min(vels, default=0)), abs(max(vels, default=0)))
            ss = report_pair(actual_key, sp_key, label, unit, records)
            suggest(label, unit, ss, vel_peak)

        print()
        print("─" * 62)
        print("  General note:")
        print("  All TORQUE_CURRENT_FOC subsystems need kS + kV feedforward.")
        print("  Without them, the motor only pushes when there is error.")
        print("  Run Phoenix Tuner X SysId or set kS/kV manually and iterate.")
        print("─" * 62)

if __name__ == '__main__':
    main()
