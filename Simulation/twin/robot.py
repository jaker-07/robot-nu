"""Robot body, drive and ultrasonic sensors.

Robot frame: origin at the midpoint of the drive axle, x forward, y to the left.
Everything is in cm, seconds and radians; motor commands are signed % power.
"""
from __future__ import annotations

import math

from .track import START_WIDTH

SOUND_US_PER_CM = 58.31          # there and back, 343 m/s


def build_hull(cal):
    """Outline of the robot seen from above (robot frame, counter-clockwise).
    The widest point is the outer tip of the 45 degree sensors."""
    fr = cal["robot.front_reach_cm"]
    rr = cal["robot.rear_reach_cm"]
    hw = cal["robot.body_half_width_cm"]
    span = cal["robot.sensor_span_cm"] / 2.0
    sx = cal["robot.side_sensor_x_cm"]
    left = [(fr, 2.3), (sx + 2.3, span - 2.5), (sx - 0.9, span), (sx - 2.8, hw),
            (-rr + 2.0, hw - 0.35), (-rr, hw - 2.1)]
    right = [(x, -y) for x, y in reversed(left)]
    return left + right


class Robot:
    def __init__(self, cal, track, rng, vary, h):
        self.track = track
        self.hull = build_hull(cal)
        self.reach = max(math.hypot(x, y) for x, y in self.hull) + 1.5
        self.wheel_track = cal["robot.track_cm"]
        ang = math.radians(cal["robot.side_sensor_angle_deg"])
        sx, sy = cal["robot.side_sensor_x_cm"], cal["robot.side_sensor_y_cm"]
        self.sensors = {"L": (sx, sy, ang), "F": (cal["robot.front_reach_cm"], 0.0, 0.0),
                        "R": (sx, -sy, -ang)}
        self.ir_x = cal["robot.ir_x_cm"]
        self.wall_r = cal["track.wall_radius_cm"]

        gain = cal["motor.speed_gain"]
        mismatch = 0.0
        if vary:
            gain *= 1.0 + rng.gauss(0.0, cal["vary.speed_pct"] / 100.0)
            mismatch = rng.gauss(0.0, cal["vary.mismatch_pct"] / 100.0)
        self.gain_l = gain * (1.0 - mismatch / 2.0)
        self.gain_r = gain * (1.0 + mismatch / 2.0)
        self.right_strength = cal["motor.right_strength"]
        self.dead_run = cal["motor.dead_run_pct"]
        self.dead_start = cal["motor.dead_start_pct"]
        self.a_run = 1.0 - math.exp(-h / cal["motor.tau_s"])
        self.a_stop = 1.0 - math.exp(-h / cal["motor.tau_stop_s"])
        self.scrub = cal["motor.turn_scrub"]
        self.h = h

        lat = rng.gauss(0.0, cal["vary.start_lat_cm"]) if vary else 0.0
        head = rng.gauss(0.0, cal["vary.start_heading_deg"]) if vary else 0.0
        lat = max(-4.0, min(4.0, lat))
        self.x = START_WIDTH / 2.0 + lat
        self.y = cal["start.axle_y_cm"]
        self.th = math.radians(90.0 + head)
        self.vl = self.vr = 0.0          # wheel ground speeds, cm/s
        self.ul = self.ur = 0.0          # commanded power, signed %
        self.speed = 0.0                 # forward speed of the axle midpoint
        self.omega = 0.0
        self.in_contact = False
        self.contact_speed = 0.0         # approach speed of the latest contact
        self.contact_point = (0.0, 0.0)

    # ---------------------------------------------------------------- drive
    def _wheel(self, v, u, gain, strength):
        eff = abs(u) * strength
        dead = self.dead_run if abs(v) > 0.5 else self.dead_start
        if eff <= dead:
            return v - v * self.a_stop
        target = gain * (eff - self.dead_run)
        if u < 0:
            target = -target
        if self.in_contact:
            target *= 0.6
        return v + (target - v) * self.a_run

    def step(self):
        """Advance one physics step. Returns True while touching a wall."""
        if self.ul == 0 and self.ur == 0 and abs(self.vl) < 0.02 and abs(self.vr) < 0.02:
            self.vl = self.vr = self.speed = self.omega = 0.0
            self.in_contact = False
            return False
        h = self.h
        vl = self.vl = self._wheel(self.vl, self.ul, self.gain_l, 1.0)
        vr = self.vr = self._wheel(self.vr, self.ur, self.gain_r, self.right_strength)
        v = (vl + vr) / 2.0
        w = (vr - vl) / self.wheel_track
        denom = abs(vl) + abs(vr)
        if denom > 1e-6:
            w *= 1.0 - (1.0 - self.scrub) * min(1.0, abs(vr - vl) / denom)
        self.th += w * h
        self.x += v * math.cos(self.th) * h
        self.y += v * math.sin(self.th) * h
        self.speed, self.omega = v, w
        self.in_contact = self._collide(v, w)
        return self.in_contact

    # ------------------------------------------------------------ collision
    def _collide(self, v, w):
        r = self.wall_r
        r2 = r * r
        reach = self.reach
        touched = False
        for _ in range(3):
            x, y = self.x, self.y
            c, s = math.cos(self.th), math.sin(self.th)
            best = 0.0
            bnx = bny = bpx = bpy = 0.0
            verts = None
            for seg in self.track.segs:
                if (x < seg[6] - reach or x > seg[8] + reach or
                        y < seg[7] - reach or y > seg[9] + reach):
                    continue
                if verts is None:
                    verts = [(x + hx * c - hy * s, y + hx * s + hy * c) for hx, hy in self.hull]
                ax, ay = seg[0], seg[1]
                ux, uy = seg[2] - ax, seg[3] - ay
                l2 = ux * ux + uy * uy
                for vx, vy in verts:
                    t = ((vx - ax) * ux + (vy - ay) * uy) / l2
                    if t < 0.0:
                        t = 0.0
                    elif t > 1.0:
                        t = 1.0
                    dx, dy = vx - (ax + t * ux), vy - (ay + t * uy)
                    d2 = dx * dx + dy * dy
                    if d2 < r2:
                        d = math.sqrt(d2)
                        if d < 1e-6:
                            continue
                        if r - d > best:
                            best = r - d
                            bnx, bny, bpx, bpy = dx / d, dy / d, vx, vy
            # wall ends and corners against the robot's sides
            for wx, wy in self.track.corners:
                if abs(wx - x) > reach or abs(wy - y) > reach:
                    continue
                lx = (wx - x) * c + (wy - y) * s
                ly = -(wx - x) * s + (wy - y) * c
                hit = self._point_vs_hull(lx, ly, r)
                if hit and hit[0] > best:
                    best = hit[0]
                    bnx = hit[1] * c - hit[2] * s
                    bny = hit[1] * s + hit[2] * c
                    bpx, bpy = wx, wy
            if best <= 0.0:
                break
            if not touched:
                rx, ry = bpx - x, bpy - y
                pvx = v * c - w * ry
                pvy = v * s + w * rx
                self.contact_speed = max(0.0, -(pvx * bnx + pvy * bny))
                self.contact_point = (bpx, bpy)
            touched = True
            self.x += bnx * best
            self.y += bny * best
            self.th += ((bpx - x) * bny - (bpy - y) * bnx) * best / 60.0
        return touched

    def _point_vs_hull(self, px, py, r):
        """Wall point (robot frame) against the outline. Returns (depth, nx, ny): how far
        and which way (robot frame) the robot must move to clear it, or None."""
        hull = self.hull
        n = len(hull)
        inside = False
        best_d2 = 1e18
        qx = qy = 0.0
        j = n - 1
        for i in range(n):
            x1, y1 = hull[j]
            x2, y2 = hull[i]
            if (y1 > py) != (y2 > py) and px < (x2 - x1) * (py - y1) / (y2 - y1) + x1:
                inside = not inside
            ux, uy = x2 - x1, y2 - y1
            t = ((px - x1) * ux + (py - y1) * uy) / (ux * ux + uy * uy)
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            cx, cy = x1 + t * ux, y1 + t * uy
            d2 = (px - cx) ** 2 + (py - cy) ** 2
            if d2 < best_d2:
                best_d2, qx, qy = d2, cx, cy
            j = i
        d = math.sqrt(best_d2)
        if d < 1e-6:
            return None
        ex, ey = (qx - px) / d, (qy - py) / d      # from the wall point to the outline
        if inside:
            return (d + r, -ex, -ey)
        if d < r:
            return (r - d, ex, ey)
        return None

    # -------------------------------------------------------------- helpers
    def world_point(self, lx, ly):
        c, s = math.cos(self.th), math.sin(self.th)
        return self.x + lx * c - ly * s, self.y + lx * s + ly * c


def ray_hit(ox, oy, dx, dy, segs, max_r):
    best = max_r
    hit = None
    for sg in segs:
        ax, ay = sg[0], sg[1]
        ex, ey = sg[2] - ax, sg[3] - ay
        den = dx * ey - dy * ex
        if -1e-9 < den < 1e-9:
            continue
        wx, wy = ax - ox, ay - oy
        t = (wx * ey - wy * ex) / den
        if t <= 0.0 or t >= best:
            continue
        u = (wx * dy - wy * dx) / den
        if 0.0 <= u <= 1.0:
            best = t
            hit = sg
    return best, hit


class Sonar:
    """Three HC-SR04 sensors.

    Each ping is a fan of rays. A ray returns an echo if the wall it meets is within its
    reach, which shrinks the further the ray is off the sensor axis and the more obliquely
    it meets the wall; the reading is the nearest returning ray. A sensor can also pick up the tail of the previous sensor's
    ping (cross-talk), more likely the shorter the pause between them.
    """
    MAX_RANGE = 400.0

    def __init__(self, cal, track, robot, rng):
        self.track = track
        self.robot = robot
        self.rng = rng
        full, edge = cal["sonar.beam_full_deg"], cal["sonar.beam_edge_deg"]
        mid = (full + edge) / 2.0
        self.rays = []
        for a, g in ((0.0, 1.0), (full / 2, 1.0), (-full / 2, 1.0), (full, 1.0), (-full, 1.0),
                     (mid, cal["sonar.edge_gain"]), (-mid, cal["sonar.edge_gain"])):
            self.rays.append((math.radians(a), g))
        self.inc_full = cal["sonar.inc_full_deg"]
        self.cos_full = math.cos(math.radians(self.inc_full))
        self.cos_max = math.cos(math.radians(87.0))
        self.edge_gain = cal["sonar.edge_gain"]
        self.graze_range = cal["sonar.graze_range_cm"]
        self.noise = cal["sonar.noise_cm"]
        self.dropout = cal["sonar.dropout"]
        self.xt_p0 = cal["sonar.xtalk_p0"]
        self.xt_len = cal["sonar.xtalk_len_cm"]
        self.rise_delay = cal["sonar.rise_delay_us"]
        self.no_echo_us = cal["sonar.no_echo_ms"] * 1000.0
        self.window = {"L": None, "F": None, "R": None}     # (rise_us, fall_us)
        self.last = {"L": -1.0, "F": -1.0, "R": -1.0}       # last true ping result, cm
        self.last_ping = None                                # (name, t_us)
        self.pings = 0

    def measure(self, name):
        rb = self.robot
        lx, ly, la = rb.sensors[name]
        ox, oy = rb.world_point(lx, ly)
        axis = rb.th + la
        segs = self.track.segs
        rng = self.rng
        if rng.random() < self.dropout:
            return None
        jitter = 1.0 + rng.gauss(0.0, 0.08)          # echo strength varies a little ping to ping
        best = None
        for a, gain in self.rays:
            dx, dy = math.cos(axis + a), math.sin(axis + a)
            t, sg = ray_hit(ox, oy, dx, dy, segs, self.MAX_RANGE)
            if sg is None or (best is not None and t >= best):
                continue
            cosi = abs(dx * sg[4] + dy * sg[5])
            if cosi <= self.cos_max:
                continue
            reach = self.MAX_RANGE
            if cosi < self.cos_full:
                # Oblique wall: matte board scatters some sound back, but the echo is weaker
                # and so is only heard from shorter range.
                q = (87.0 - math.degrees(math.acos(cosi))) / (87.0 - self.inc_full)
                reach = self.graze_range + (self.MAX_RANGE - self.graze_range) * q
            if t <= reach * gain * jitter:
                best = t
        if best is not None:
            best = max(2.0, best + rng.gauss(0.0, self.noise))
        return best

    def trigger(self, name, t_us):
        """TRIG pulse finished at t_us."""
        win = self.window[name]
        if win is not None and t_us < win[1]:
            return                                   # still busy: the pulse is ignored
        d = self.measure(name)
        if self.last_ping is not None and self.last_ping[0] != name:
            path = (t_us - self.last_ping[1]) / SOUND_US_PER_CM * 2.0   # cm of sound travel
            if self.rng.random() < self.xt_p0 * math.exp(-path / self.xt_len):
                stray = self.rng.uniform(2.0, 60.0)
                if d is None or stray < d:
                    d = stray
        self.last_ping = (name, t_us)
        self.pings += 1
        rise = t_us + self.rise_delay
        fall = rise + (d * SOUND_US_PER_CM if d is not None else self.no_echo_us)
        self.window[name] = (rise, fall)
        self.last[name] = -1.0 if d is None else d

    def echo_level(self, name, t_us):
        win = self.window[name]
        return 1 if (win is not None and win[0] <= t_us < win[1]) else 0
