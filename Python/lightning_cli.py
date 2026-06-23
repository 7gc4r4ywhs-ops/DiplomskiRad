#!/usr/bin/env python3
"""
Lightning Detection Network — Laptop CLI
Connects to master node via serial, provides real-time dashboard
and lightning strike triangulation.

Usage: python3 lightning_cli.py --port /dev/ttyUSB0 [--baud 115200]

Requirements: pip install pyserial
"""

import argparse
import curses
import json
import math
import os
import re
import serial
import threading
import time
from datetime import datetime
from collections import deque

# ── Config ────────────────────────────────────────────────────────────────────

DATA_FILE        = "node_locations.json"
LOG_FILE         = f"lightning_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
STRIKE_LOG_FILE  = f"strikes_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
STRIKE_WINDOW_S  = 5      # seconds — reports within this window = same strike
SENSOR_TOLERANCE = 1.0    # km — widen intersection area for sensor precision
EARTH_RADIUS_KM  = 6371.0

# ── Data store ────────────────────────────────────────────────────────────────

node_locations = {}   # {nodeId: {"lat": float, "lon": float, "name": str}}
topology       = {}   # {nodeId: {"upstream": int, "downstream": int}}
all_reports     = []  # every report ever seen: {"nodeId", "distance_km", "timestamp", "received_at"}
processed_strikes = {} # {strike_key: {"reports": [...], "result": str}} — already triangulated
strike_log     = []   # list of processed strikes
serial_log     = deque(maxlen=200)  # raw serial lines
event_log      = deque(maxlen=100)  # parsed events for display

lock = threading.Lock()

# ── Persistence ───────────────────────────────────────────────────────────────

def load_locations():
    global node_locations
    if os.path.exists(DATA_FILE):
        with open(DATA_FILE, "r") as f:
            node_locations = json.load(f)
        # Convert keys to int
        node_locations = {int(k): v for k, v in node_locations.items()}

def save_locations():
    with open(DATA_FILE, "w") as f:
        json.dump({str(k): v for k, v in node_locations.items()}, f, indent=2)

# ── Logging ───────────────────────────────────────────────────────────────────

def log(line):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a") as f:
        f.write(f"[{timestamp}] {line}\n")

def log_strike(line):
    """Log only to the lightning-specific log file."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(STRIKE_LOG_FILE, "a") as f:
        f.write(f"[{timestamp}] {line}\n")

# ── Geo math ──────────────────────────────────────────────────────────────────

def haversine(lat1, lon1, lat2, lon2):
    """Distance in km between two lat/lon points."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return 2 * EARTH_RADIUS_KM * math.asin(math.sqrt(a))

def km_to_deg_lat(km):
    return km / 111.32

def km_to_deg_lon(km, lat):
    return km / (111.32 * math.cos(math.radians(lat)))

def circle_intersections(lat1, lon1, r1, lat2, lon2, r2):
    """
    Find intersection points of two circles on Earth surface.
    Centers at (lat1,lon1) and (lat2,lon2), radii r1 and r2 in km.
    Returns list of (lat, lon) intersection points, empty if none.
    """
    d = haversine(lat1, lon1, lat2, lon2)

    if d > r1 + r2 + SENSOR_TOLERANCE * 2:
        return []  # circles too far apart
    if d < abs(r1 - r2) - SENSOR_TOLERANCE * 2:
        return []  # one circle inside the other
    if d < 0.001:
        return []  # same center

    # Work in local flat-earth approximation (good for small distances)
    # Convert to x,y (km) with node1 at origin
    bearing = math.atan2(
        math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2)),
        math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) -
        math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) *
        math.cos(math.radians(lon2 - lon1))
    )
    x2 = d * math.sin(bearing)
    y2 = d * math.cos(bearing)

    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h_sq = r1**2 - a**2
    if h_sq < 0:
        return []  # circles don't intersect (one inside the other)
    h = math.sqrt(h_sq)

    # Midpoint
    mx = a * x2 / d
    my = a * y2 / d

    # Two intersection points in local coords
    px1 = mx + h * y2 / d
    py1 = my - h * x2 / d
    px2 = mx - h * y2 / d
    py2 = my + h * x2 / d

    # Convert back to lat/lon
    def local_to_latlon(px, py):
        dlat = km_to_deg_lat(py)
        dlon = km_to_deg_lon(px, lat1)
        return lat1 + dlat, lon1 + dlon

    p1 = local_to_latlon(px1, py1)
    p2 = local_to_latlon(px2, py2)

    if h < 0.001:
        return [p1]  # tangent — one intersection
    return [p1, p2]

def find_node_at(lat, lon, tolerance_km):
    """Check if any known node is within tolerance_km of (lat, lon)."""
    with lock:
        for node_id, loc in node_locations.items():
            d = haversine(lat, lon, loc["lat"], loc["lon"])
            if d <= tolerance_km:
                return node_id, d
    return None, None

# ── Strike processing ─────────────────────────────────────────────────────────

def process_report(node_id, distance_km, timestamp):
    """
    Add a new lightning report. Reports are correlated by packet timestamp
    (within STRIKE_WINDOW_S). If a late report correlates with an already
    processed strike, the strike is re-triangulated with the new data.
    """
    now = time.time()
    report = {
        "nodeId":      node_id,
        "distance_km": distance_km,
        "timestamp":   timestamp,
        "received_at": now
    }

    with lock:
        all_reports.append(report)

    # Find all reports correlated with this one by packet timestamp
    with lock:
        correlated = [r for r in all_reports
                      if abs(r["timestamp"] - timestamp) <= STRIKE_WINDOW_S]

    if len(correlated) < 2:
        return  # need at least 2 reports to triangulate

    # Strike key — group by rounded timestamp window
    # Use the minimum timestamp in the correlated group as a stable key
    strike_key = min(r["timestamp"] for r in correlated)

    with lock:
        already = strike_key in processed_strikes
        prev_count = len(processed_strikes[strike_key]["reports"]) if already else 0

    # Only (re)process if this is new or we have more reports than before
    if already and len(correlated) <= prev_count:
        return

    if already:
        with lock:
            event_log.append(f"  ↻ New report for strike — reprocessing with {len(correlated)} reports")

    triangulate(correlated, strike_key)

def triangulate(reports, strike_key):
    """
    Triangulate from all correlated reports.
    Computes circle intersections for every node pair, clusters the
    candidate points, and reports the location where most pairs agree.
    Result is stored in processed_strikes[strike_key].
    """
    # Check all nodes have known locations
    with lock:
        locs = {r["nodeId"]: node_locations.get(r["nodeId"]) for r in reports}

    ts = datetime.fromtimestamp(reports[0]["timestamp"]).strftime("%H:%M:%S")
    nodes_str = ", ".join(f"N{r['nodeId']}({r['distance_km']}km)" for r in reports)

    with lock:
        event_log.append(f"━━━ LIGHTNING STRIKE @ {ts} ━━━")
        event_log.append(f"  Reports: {nodes_str}")
    log(f"LIGHTNING STRIKE @ {ts} — Reports: {nodes_str}")
    log_strike(f"LIGHTNING STRIKE @ {ts} — Reports: {nodes_str}")

    if any(v is None for v in locs.values()):
        missing = [r["nodeId"] for r in reports if locs.get(r["nodeId"]) is None]
        msg = f"  Cannot triangulate — missing location for nodes: {missing}"
        with lock:
            event_log.append(msg)
            processed_strikes[strike_key] = {"reports": list(reports), "result": msg}
        log(msg)
        log_strike(msg)
        return

    # Compute intersections for every pair of reports
    candidates = []  # list of (lat, lon)
    for i in range(len(reports)):
        for j in range(i + 1, len(reports)):
            ri, rj = reports[i], reports[j]
            li, lj = locs[ri["nodeId"]], locs[rj["nodeId"]]
            pts = circle_intersections(
                li["lat"], li["lon"], ri["distance_km"],
                lj["lat"], lj["lon"], rj["distance_km"]
            )
            candidates.extend(pts)

    if not candidates:
        msg = "  No intersection found (circles don't overlap)"
        with lock:
            event_log.append(msg)
            processed_strikes[strike_key] = {"reports": list(reports), "result": msg}
        log(msg)
        log_strike(msg)
        return

    # Cluster candidate points — group points within CLUSTER_RADIUS km
    CLUSTER_RADIUS = 2.0  # km
    clusters = []
    for pt in candidates:
        placed = False
        for cluster in clusters:
            if haversine(pt[0], pt[1], cluster[0][0], cluster[0][1]) <= CLUSTER_RADIUS:
                cluster.append(pt)
                placed = True
                break
        if not placed:
            clusters.append([pt])

    # Best cluster = the one with most agreeing points
    best = max(clusters, key=len)
    avg_lat = sum(p[0] for p in best) / len(best)
    avg_lon = sum(p[1] for p in best) / len(best)

    confidence = len(best)
    total      = len(candidates)
    msg = f"  Strike location: {avg_lat:.6f}, {avg_lon:.6f}  ({confidence}/{total} pairs agree)"
    with lock:
        event_log.append(msg)
    log(msg)
    log_strike(msg)

    # Check if any node is within 1km of the strike location
    node_id, dist = find_node_at(avg_lat, avg_lon, SENSOR_TOLERANCE)
    if node_id is not None:
        result_line = f"  ⚡ NODE {node_id} STRUCK — {dist:.2f}km from estimated location"
    else:
        result_line = "  No node within 1km of strike location"

    with lock:
        event_log.append(result_line)
        processed_strikes[strike_key] = {
            "reports": list(reports),
            "result": f"{msg}\n{result_line}"
        }
    log(result_line)
    log_strike(result_line)

# ── Serial parser ─────────────────────────────────────────────────────────────

DATA_RE     = re.compile(r'\[DATA\] Dist: (\d+) km, Time: \S+ \((\d+)\), From: (\d+)')
TOPOLOGY_RE = re.compile(r'Node (\d+): UP=(\d+) DOWN=(\d+)')
ENROLLED_RE = re.compile(r'Node (\d+) enrolled')

def parse_line(line):
    line = line.strip()
    if not line:
        return

    with lock:
        serial_log.append(line)

    log(f"[SERIAL] {line}")

    # DATA packet received
    m = DATA_RE.search(line)
    if m:
        dist_km   = int(m.group(1))
        timestamp = int(m.group(2))
        node_id   = int(m.group(3))
        msg = f"[DATA] Node {node_id}: {dist_km} km @ {datetime.fromtimestamp(timestamp).strftime('%H:%M:%S')}"
        with lock:
            event_log.append(msg)
        process_report(node_id, dist_km, timestamp)
        return

    # Topology entry
    m = TOPOLOGY_RE.search(line)
    if m:
        node_id = int(m.group(1))
        up      = int(m.group(2))
        down    = int(m.group(3))
        with lock:
            topology[node_id] = {"upstream": up, "downstream": down}
        return

# ── Serial reader thread ──────────────────────────────────────────────────────

def serial_reader(ser):
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="replace")
            if line:
                parse_line(line)
        except Exception as e:
            with lock:
                serial_log.append(f"[ERROR] Serial read: {e}")
            time.sleep(0.1)

# ── Command processing ────────────────────────────────────────────────────────

def process_command(cmd, ser):
    """Process CLI command — either local or forward to master serial."""
    parts = cmd.strip().split()
    if not parts:
        return ""

    # Local commands
    if parts[0] == "setlocation" and len(parts) >= 4:
        try:
            node_id = int(parts[1])
            lat     = float(parts[2])
            lon     = float(parts[3])
            name    = " ".join(parts[4:]) if len(parts) > 4 else f"Node {node_id}"
            with lock:
                node_locations[node_id] = {"lat": lat, "lon": lon, "name": name}
            save_locations()
            return f"Location set: Node {node_id} → {lat:.6f}, {lon:.6f} ({name})"
        except ValueError:
            return "Usage: setlocation <nodeId> <lat> <lon> [name]"

    if parts[0] == "showlocations":
        with lock:
            locs = dict(node_locations)
        if not locs:
            return "No locations set"
        lines = ["Node locations:"]
        for nid, loc in sorted(locs.items()):
            lines.append(f"  Node {nid}: {loc['lat']:.6f}, {loc['lon']:.6f}  ({loc.get('name', '')})")
        return "\n".join(lines)

    if parts[0] == "removelocation" and len(parts) >= 2:
        try:
            node_id = int(parts[1])
            with lock:
                node_locations.pop(node_id, None)
            save_locations()
            return f"Location removed for node {node_id}"
        except ValueError:
            return "Usage: removelocation <nodeId>"

    if parts[0] == "strikes":
        with lock:
            events = list(event_log)
        return "\n".join(events[-20:]) if events else "No strikes recorded"

    if parts[0] == "help":
        return (
            "=== Local commands ===\n"
            "  setlocation <id> <lat> <lon> [name]  - Set node GPS location\n"
            "  removelocation <id>                  - Remove node location\n"
            "  showlocations                        - Show all node locations\n"
            "  strikes                              - Show recent strike events\n"
            "  help                                 - Show this help\n"
            "=== Master commands (forwarded via serial) ===\n"
            "  config, topology, removenode <id>, syncnetwork, syncclock\n"
            "  setssid <s>, setwifipass <p>, showwifi, settz <tz>\n"
            "  setid <id>, setup <id>, setdown <id>, setenrolled <0|1>\n"
            "  sdata <dst> <km>, factoryreset\n"
        )

    # Forward everything else to master via serial
    try:
        ser.write((cmd.strip() + "\n").encode("utf-8"))
        return f"→ Sent to master: {cmd.strip()}"
    except Exception as e:
        return f"[ERROR] Serial write: {e}"

# ── Curses UI ─────────────────────────────────────────────────────────────────

def draw_ui(stdscr, ser):
    curses.curs_set(1)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN,  -1)  # serial log
    curses.init_pair(2, curses.COLOR_YELLOW, -1)  # events
    curses.init_pair(3, curses.COLOR_CYAN,   -1)  # header
    curses.init_pair(4, curses.COLOR_RED,    -1)  # errors/strikes
    curses.init_pair(5, curses.COLOR_WHITE,  -1)  # normal

    stdscr.nodelay(True)
    stdscr.keypad(True)

    cmd_buf = ""
    cmd_history = []
    hist_idx = -1
    output_msg = ""
    output_timer = 0

    while True:
        h, w = stdscr.getmaxyx()

        # Layout: top half = serial log, bottom half = events, bottom = input
        serial_h = h // 2 - 1
        event_h  = h - serial_h - 4
        input_y  = h - 3

        stdscr.erase()

        # ── Header ────────────────────────────────────────────────────────
        header = f" Lightning Detection CLI  |  Log: {LOG_FILE}  |  Strikes: {STRIKE_LOG_FILE} "
        stdscr.attron(curses.color_pair(3) | curses.A_BOLD)
        stdscr.addstr(0, 0, header[:w-1].ljust(w-1))
        stdscr.attroff(curses.color_pair(3) | curses.A_BOLD)

        # ── Serial log pane ───────────────────────────────────────────────
        stdscr.attron(curses.color_pair(3))
        stdscr.addstr(1, 0, "─── Serial Output " + "─" * max(0, w - 19))
        stdscr.attroff(curses.color_pair(3))

        with lock:
            lines = list(serial_log)
        visible = lines[-(serial_h):]
        for i, line in enumerate(visible):
            y = 2 + i
            if y >= serial_h + 2:
                break
            try:
                color = curses.color_pair(4) if "[ERROR]" in line else curses.color_pair(1)
                stdscr.addstr(y, 0, line[:w-1], color)
            except curses.error:
                pass

        # ── Events pane ───────────────────────────────────────────────────
        events_y = serial_h + 2
        stdscr.attron(curses.color_pair(3))
        stdscr.addstr(events_y, 0, "─── Lightning Events " + "─" * max(0, w - 21))
        stdscr.attroff(curses.color_pair(3))

        with lock:
            evts = list(event_log)
        visible_evts = evts[-(event_h):]
        for i, evt in enumerate(visible_evts):
            y = events_y + 1 + i
            if y >= input_y - 1:
                break
            try:
                color = curses.color_pair(4) if "STRIKE" in evt else curses.color_pair(2)
                stdscr.addstr(y, 0, evt[:w-1], color)
            except curses.error:
                pass

        # ── Input area ────────────────────────────────────────────────────
        stdscr.attron(curses.color_pair(3))
        stdscr.addstr(input_y - 1, 0, "─" * max(0, w - 1))
        stdscr.attroff(curses.color_pair(3))

        # Output message (clears after 3s)
        if output_msg and time.time() - output_timer < 3:
            try:
                stdscr.addstr(input_y, 0, output_msg[:w-1], curses.color_pair(5))
            except curses.error:
                pass
        else:
            output_msg = ""

        prompt = "> "
        try:
            stdscr.addstr(input_y + 1, 0, prompt + cmd_buf, curses.color_pair(5))
            stdscr.move(input_y + 1, len(prompt) + len(cmd_buf))
        except curses.error:
            pass

        stdscr.refresh()

        # ── Input handling ────────────────────────────────────────────────
        try:
            ch = stdscr.getch()
        except curses.error:
            ch = -1

        if ch == -1:
            time.sleep(0.05)
            continue

        if ch in (curses.KEY_ENTER, ord('\n'), ord('\r')):
            if cmd_buf.strip():
                cmd_history.append(cmd_buf)
                hist_idx = -1
                result = process_command(cmd_buf, ser)
                if result:
                    output_msg   = result
                    output_timer = time.time()
                    with lock:
                        for line in result.split("\n"):
                            serial_log.append(line)
                    log(f"[CMD] {cmd_buf} → {result}")
                cmd_buf = ""

        elif ch in (curses.KEY_BACKSPACE, 127, 8):
            cmd_buf = cmd_buf[:-1]

        elif ch == curses.KEY_UP:
            if cmd_history:
                hist_idx = max(0, (hist_idx - 1) if hist_idx >= 0 else len(cmd_history) - 1)
                cmd_buf = cmd_history[hist_idx]

        elif ch == curses.KEY_DOWN:
            if hist_idx >= 0:
                hist_idx += 1
                if hist_idx >= len(cmd_history):
                    hist_idx = -1
                    cmd_buf = ""
                else:
                    cmd_buf = cmd_history[hist_idx]

        elif ch == 27:  # ESC
            break

        elif 32 <= ch <= 126:
            cmd_buf += chr(ch)

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Lightning Detection CLI")
    parser.add_argument("--port",  required=True, help="Serial port (e.g. /dev/ttyUSB0 or COM3)")
    parser.add_argument("--baud",  type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    load_locations()
    log(f"=== Session started — port: {args.port} baud: {args.baud} ===")
    log_strike(f"=== Session started — port: {args.port} baud: {args.baud} ===")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"Connected to {args.port} @ {args.baud} baud")
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    # Start serial reader thread
    t = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
    t.start()

    # Start curses UI
    try:
        curses.wrapper(draw_ui, ser)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        log("=== Session ended ===")
        log_strike("=== Session ended ===")
        print(f"\nSession logged to: {LOG_FILE}")
        print(f"Strike log: {STRIKE_LOG_FILE}")

if __name__ == "__main__":
    main()