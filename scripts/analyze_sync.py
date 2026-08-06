import argparse
import csv
import os
import re
import statistics
import sys

DEFAULT_FRAME_RATE = 30.0             # FPS; grouping tolerance = 1e6 / fps / 2 us
DEFAULT_TSP_RANGE_THRESHOLD = 2000.0  # us; in-group range >= this -> abnormal
DEFAULT_TIMESTAMP_SOURCE = "auto"     # global | device | auto

CSV_NAME_RE = re.compile(r"^sync_(depth|color)_dev(\d+)_(.+)\.csv$", re.IGNORECASE)


# --------------------------------------------------------------------------- #
# Data structures
# --------------------------------------------------------------------------- #
class FrameRecord:
    __slots__ = ("row_id", "sw_frame_num", "hw_frame_num",
                 "system_ts", "device_ts", "global_ts")

    def __init__(self, row_id, sw_frame_num, hw_frame_num,
                 system_ts, device_ts, global_ts):
        self.row_id = row_id
        self.sw_frame_num = sw_frame_num
        self.hw_frame_num = hw_frame_num
        self.system_ts = system_ts
        self.device_ts = device_ts
        self.global_ts = global_ts

    def ts(self, source):
        return self.global_ts if source == "global" else self.device_ts


class DeviceFrames:
    def __init__(self, index, sn, sensor):
        self.index = index
        self.sn = sn
        self.sensor = sensor
        self.frames = []
        self._sorted = []
        self._ts = []
        self.hw_valid = True
        self.sw_valid = True

    def sort(self, ts_source):
        self.frames.sort(key=lambda r: r.ts(ts_source))
        self._sorted = self.frames
        self._ts = [r.ts(ts_source) for r in self._sorted]

    def is_ts_valid(self, ts_source):
        if not self.frames:
            return False
        return any(r.ts(ts_source) != 0 for r in self.frames)

    def check_frame_num_validity(self):
        if self.frames:
            self.hw_valid = any(r.hw_frame_num != -1 for r in self.frames)
            self.sw_valid = any(r.sw_frame_num != -1 for r in self.frames)

    def nearest_from(self, target_ts, start_idx, max_diff):
        """Find the frame nearest to target_ts at index >= start_idx. Returns (idx, diff) or (-1, None)."""
        arr = self._ts
        n = len(arr)
        last_below = -1
        j = start_idx
        while j < n and arr[j] < target_ts:
            if arr[j] >= target_ts - max_diff:
                last_below = j
            j += 1
        below_diff = (target_ts - arr[last_below]) if last_below != -1 else None
        above_diff = (arr[j] - target_ts) if j < n and arr[j] <= target_ts + max_diff else None
        if below_diff is None and above_diff is None:
            return -1, None
        if below_diff is None:
            return j, above_diff
        if above_diff is None:
            return last_below, below_diff
        return (last_below, below_diff) if below_diff <= above_diff else (j, above_diff)


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #
def discover_csv(data_dir):
    result = {"depth": {}, "color": {}}
    found = 0
    for name in os.listdir(data_dir):
        m = CSV_NAME_RE.match(name)
        if not m:
            continue
        sensor = m.group(1).lower()
        index = int(m.group(2))
        sn = m.group(3)
        df = DeviceFrames(index, sn, sensor)
        path = os.path.join(data_dir, name)
        try:
            with open(path, "r", encoding="utf-8", newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if row.get("global_ts_us") in (None, "") and row.get("device_ts_us") in (None, ""):
                        continue
                    df.frames.append(FrameRecord(
                        int(row["row_id"]) if row.get("row_id") not in (None, "") else 0,
                        int(row["sw_frame_num"]) if row.get("sw_frame_num") not in (None, "") else -1,
                        int(row["hw_frame_num"]) if row.get("hw_frame_num") not in (None, "") else -1,
                        int(row["system_ts_us"]) if row.get("system_ts_us") not in (None, "") else 0,
                        int(row["device_ts_us"]) if row.get("device_ts_us") not in (None, "") else 0,
                        int(row["global_ts_us"]) if row.get("global_ts_us") not in (None, "") else 0,
                    ))
        except (OSError, ValueError, KeyError) as e:
            print("WARN: failed to read %s: %s" % (name, e))
            continue
        df.check_frame_num_validity()
        result[sensor][index] = df
        found += 1
    return result, found


def resolve_ts_source(requested, devices, lines):
    """Resolve the time base from the request and data validity across all devices."""
    global_valid = all(d.is_ts_valid("global") for d in devices)
    device_valid = all(d.is_ts_valid("device") for d in devices)
    chosen = requested
    notes = []
    if requested == "auto":
        if global_valid:
            chosen = "global"
        elif device_valid:
            chosen = "device"
            notes.append("Auto-degraded to device timestamps (global_ts_us all 0, device may not support global timestamp)")
        else:
            chosen = "device"
            notes.append("WARN: both global and device timestamps are abnormal; still matching on device")
    elif requested == "global" and not global_valid:
        invalid = [d.index for d in devices if not d.is_ts_valid("global")]
        notes.append("WARN: global selected but dev%s has all-zero global_ts_us" % invalid)
    elif requested == "device" and not device_valid:
        notes.append("WARN: device timestamps are abnormal")
    for n in notes:
        lines.append("  " + n)
    return chosen


def group_synced_frames(devices, ts_source, half_gap_us):
    """Greedy grouping: each round picks the earliest remaining frame as the anchor
    and matches all devices within the half-frame interval.
    Returns (matched_groups, skipped); each group is {dev_index: FrameRecord}."""
    devices = sorted(devices, key=lambda d: d.index)
    n = len(devices)
    ptr = [0] * n
    groups = []
    skipped = 0

    while True:
        anchor = -1
        anchor_ts = None
        for i in range(n):
            if ptr[i] < len(devices[i]._ts):
                t = devices[i]._ts[ptr[i]]
                if anchor_ts is None or t < anchor_ts:
                    anchor_ts = t
                    anchor = i
        if anchor == -1:
            break

        match = [None] * n
        match[anchor] = ptr[anchor]
        ok = True
        for i in range(n):
            if i == anchor:
                continue
            idx, diff = devices[i].nearest_from(anchor_ts, ptr[i], half_gap_us)
            if idx < 0:
                ok = False
                break
            match[i] = idx
        if not ok:
            skipped += 1
            ptr[anchor] += 1
            continue

        group = {}
        for i in range(n):
            group[devices[i].index] = devices[i]._sorted[match[i]]
            ptr[i] = match[i] + 1
        groups.append(group)

    return groups, skipped


def write_matched_csv(path, devices, groups, ts_source):
    devices = sorted(devices, key=lambda d: d.index)
    header = ["group_id", "ts_source", "Global(range)", "Device(range)"]
    for d in devices:
        prefix = "dev%d_%s" % (d.index, d.sensor)
        header += [prefix + "_swfrm",
                   prefix + "_hwfrm",
                   prefix + "_systemts",
                   prefix + "_globalts",
                   prefix + "_devicets"]
    with open(path, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for gid, group in enumerate(groups):
            global_list = [group[d.index].global_ts for d in devices]
            device_list = [group[d.index].device_ts for d in devices]
            g_range = max(global_list) - min(global_list)
            d_range = max(device_list) - min(device_list)
            row = [gid, ts_source, g_range, d_range]
            for d in devices:
                r = group[d.index]
                row += [r.sw_frame_num, r.hw_frame_num, r.system_ts, r.global_ts, r.device_ts]
            w.writerow(row)


def analyze_sensor(sensor, devices_map, ts_source, half_gap_us, threshold, lines):
    lines.append("\n========== Sensor: %s ==========" % sensor.upper())
    devices = sorted(devices_map.values(), key=lambda x: x.index)
    if len(devices) < 2:
        lines.append("    Fewer than 2 devices, cannot evaluate sync, skip")
        return None

    for d in devices:
        lines.append("    dev%d(%s): %d frames" % (d.index, d.sn, len(d.frames)))

    duration_s = 0.0
    all_ts = [r.ts(ts_source) for d in devices for r in d.frames]
    if all_ts:
        duration_s = (max(all_ts) - min(all_ts)) / 1e6
        if duration_s > 0:
            min_frames = min(len(d.frames) for d in devices)
            lines.append("    Duration: %.3f s  (~%.2f fps)" % (duration_s, min_frames / duration_s))

    groups, skipped = group_synced_frames(devices, ts_source, half_gap_us)
    total_anchors = len(groups) + skipped
    completeness = 100.0 * len(groups) / total_anchors if total_anchors else 0.0
    lines.append("    Completeness      : %.1f%% (%d groups, %d drops)" % (completeness, len(groups), skipped))

    if not groups:
        lines.append("    (no complete groups, cannot evaluate)")
        return None

    ranges = []
    for g in groups:
        ts_list = [g[d.index].ts(ts_source) for d in devices]
        ranges.append(max(ts_list) - min(ts_list))
    abnormal = sum(1 for r in ranges if r >= threshold)
    lines.append("    Abnormal (>= %.0fus): %.1f%% (%d / %d)" %
                 (threshold, 100.0 * abnormal / len(groups), abnormal, len(groups)))

    return groups


# --------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(
        description="Multi-device sync timestamp CSV analyzer",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("data_dir", nargs="?", default=None,
                        help="Directory with sync_*_dev*_<SN>.csv (default: current directory)")
    parser.add_argument("--fps", type=float, default=DEFAULT_FRAME_RATE,
                        help="Frame rate; grouping tolerance = half frame (default %.0f)" % DEFAULT_FRAME_RATE)
    parser.add_argument("--threshold", type=float, default=DEFAULT_TSP_RANGE_THRESHOLD,
                        help="Abnormal in-group range threshold in us (default %.0f)" % DEFAULT_TSP_RANGE_THRESHOLD)
    parser.add_argument("--ts-source", choices=["global", "device", "auto"],
                        default=DEFAULT_TIMESTAMP_SOURCE,
                        help="Time base: global/device/auto (default %s)" % DEFAULT_TIMESTAMP_SOURCE)
    parser.add_argument("--output", default=None,
                        help="Output directory for matched CSV (default: <data_dir>/analysis)")
    args = parser.parse_args()

    data_dir = os.path.abspath(args.data_dir) if args.data_dir else os.getcwd()
    if not os.path.isdir(data_dir):
        print("ERROR: directory does not exist: %s" % data_dir)
        return 1

    fps = args.fps
    threshold = args.threshold
    ts_source = args.ts_source
    if fps <= 0 or fps >= 1000:
        print("ERROR: invalid frameRate=%.1f" % fps)
        return 1

    half_gap_us = 1000000.0 / fps / 2.0
    base_out = args.output or data_dir
    out_dir = os.path.join(base_out, "analysis")
    os.makedirs(out_dir, exist_ok=True)

    print("=" * 60)
    print("Multi-device sync CSV analysis")
    print("  Data dir   : %s" % data_dir)
    print("  Frame rate : %.1f fps (grouping tolerance half-frame %.0f us)" % (fps, half_gap_us))
    print("  Threshold  : %.0f us" % threshold)
    print("  Time base  : %s" % ts_source)
    print("=" * 60)

    sensors_data, found = discover_csv(data_dir)
    if found == 0:
        print("ERROR: no sync_*_dev*_<SN>.csv found under %s" % data_dir)
        return 1

    lines = []
    for sensor in ("depth", "color"):
        devices = sensors_data[sensor]
        if not devices:
            lines.append("\n========== Sensor: %s ==========" % sensor.upper())
            lines.append("    No data, skip")
            continue
        chosen_ts = resolve_ts_source(ts_source, list(devices.values()), lines)
        for d in devices.values():
            d.sort(chosen_ts)
        groups = analyze_sensor(sensor, devices, chosen_ts, half_gap_us, threshold, lines)
        if groups:
            out_csv = os.path.join(out_dir, "sync_matched_%s.csv" % sensor)
            write_matched_csv(out_csv, list(devices.values()), groups, chosen_ts)
            lines.append("    Matched CSV: %s (%d groups)" % (out_csv, len(groups)))

    print("\n" + "\n".join(lines))
    print("\nMatched CSVs written to: %s" % out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())