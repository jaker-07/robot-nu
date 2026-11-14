"""The AE1 course, built from the wall lengths and angles dimensioned in the brief.

The brief's numbers do not give a constant-width corridor: the five legs come out
30.0 / 30.0 / 25.4 / 23.1 / 35.8 cm wide. The twin keeps them as dimensioned.

World frame: cm, origin at the left end of the start line, x to the right, y up
(the robot starts driving in +y).
"""
from __future__ import annotations

import math

# (length cm, heading deg) for each wall, from the start line to the far end.
OUTER = ((80.0, 90.0), (93.0, 0.0), (84.0, -60.0), (63.0, 0.0), (70.0, 60.0))
INNER = ((50.0, 90.0), (51.0, 0.0), (76.0, -60.0), (107.0, 0.0), (81.0, 60.0))
START_WIDTH = 30.0


def _seg(a, b, kind):
    ax, ay = a
    bx, by = b
    length = math.hypot(bx - ax, by - ay) or 1e-9
    nx, ny = -(by - ay) / length, (bx - ax) / length
    return (ax, ay, bx, by, nx, ny,
            min(ax, bx), min(ay, by), max(ax, bx), max(ay, by), kind)


class Track:
    def __init__(self, cal, rng=None, vary=False):
        sl = cal["vary.wall_len_cm"] if vary else 0.0
        sa = cal["vary.wall_ang_deg"] if vary else 0.0

        def chain(p0, spec):
            pts = [p0]
            x, y = p0
            for length, heading in spec:
                if vary:
                    length += rng.gauss(0.0, sl)
                    heading += rng.gauss(0.0, sa)
                x += length * math.cos(math.radians(heading))
                y += length * math.sin(math.radians(heading))
                pts.append((x, y))
            return pts

        self.outer = chain((0.0, 0.0), OUTER)
        self.inner = chain((START_WIDTH, 0.0), INNER)

        # Task Decider: a box whose near face starts on the outer wall line, gap_cm past
        # the end of the outer wall, and reaches width_cm across the corridor.
        ox, oy = self.outer[-1]
        px, py = self.outer[-2]
        ln = math.hypot(ox - px, oy - py)
        dx, dy = (ox - px) / ln, (oy - py) / ln          # driving direction of the last leg
        nx, ny = dy, -dx                                  # across the corridor, outer -> inner
        g, w, dep = cal["decider.gap_cm"], cal["decider.width_cm"], cal["decider.depth_cm"]
        off = cal["decider.offset_cm"]
        a = (ox + g * dx + off * nx, oy + g * dy + off * ny)
        b = (a[0] + w * nx, a[1] + w * ny)
        c = (b[0] + dep * dx, b[1] + dep * dy)
        d = (a[0] + dep * dx, a[1] + dep * dy)
        self.decider = [a, b, c, d]
        self.end_dir = (dx, dy)
        self.end_normal = (nx, ny)
        self.s_face = a[0] * dx + a[1] * dy               # face plane position along end_dir
        self.face_a = a
        self.face_width = w

        self.segs = []
        for pts in (self.outer, self.inner):
            for i in range(len(pts) - 1):
                self.segs.append(_seg(pts[i], pts[i + 1], 0))
        for i in range(4):
            self.segs.append(_seg(self.decider[i], self.decider[(i + 1) % 4], 1))
        self.corners = list(self.outer) + list(self.inner) + list(self.decider)

        # Centreline: midpoints of matching wall vertices, extended to the Decider face.
        mids = [((o[0] + i[0]) / 2.0, (o[1] + i[1]) / 2.0) for o, i in zip(self.outer, self.inner)]
        ex, ey = mids[-1]
        ext = self.s_face - (ex * dx + ey * dy)
        mids[-1] = (ex + ext * dx, ey + ext * dy)
        self.centre = mids
        self.cum = [0.0]
        for i in range(len(mids) - 1):
            self.cum.append(self.cum[-1] + math.hypot(mids[i + 1][0] - mids[i][0],
                                                      mids[i + 1][1] - mids[i][1]))
        self.length = self.cum[-1]

        # Corridor width of each leg (perpendicular distance between its two walls).
        self.widths = []
        for i in range(len(self.outer) - 1):
            s = self.segs[i]
            mx = (self.inner[i][0] + self.inner[i + 1][0]) / 2.0
            my = (self.inner[i][1] + self.inner[i + 1][1]) / 2.0
            self.widths.append(abs((mx - s[0]) * s[4] + (my - s[1]) * s[5]))

    def project(self, x, y):
        """Nearest point on the centreline -> (distance along it, signed sideways offset
        (+ = left of the driving direction), leg index)."""
        best = None
        c = self.centre
        for i in range(len(c) - 1):
            ax, ay = c[i]
            bx, by = c[i + 1]
            ux, uy = bx - ax, by - ay
            ln = math.hypot(ux, uy)
            ux, uy = ux / ln, uy / ln
            t = (x - ax) * ux + (y - ay) * uy
            tc = 0.0 if t < 0 else (ln if t > ln else t)
            qx, qy = ax + tc * ux, ay + tc * uy
            d2 = (x - qx) ** 2 + (y - qy) ** 2
            if best is None or d2 < best[0]:
                lat = (x - ax) * (-uy) + (y - ay) * ux
                best = (d2, self.cum[i] + tc, lat, i)
        return best[1], best[2], best[3]

    def to_json(self):
        r = lambda p: [round(p[0], 2), round(p[1], 2)]
        return {
            "outer": [r(p) for p in self.outer],
            "inner": [r(p) for p in self.inner],
            "decider": [r(p) for p in self.decider],
            "centre": [r(p) for p in self.centre],
            "widths": [round(w, 1) for w in self.widths],
            "length": round(self.length, 1),
            "start_width": START_WIDTH,
        }
