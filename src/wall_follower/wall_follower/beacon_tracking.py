"""Pure geometry and spatial tracking; never publishes motion or TF."""
from dataclasses import dataclass, field
from collections import Counter
import math
import numpy as np


def matrix(transform):
    t, q = transform.translation, transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    m = np.eye(4)
    m[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    m[:3, 3] = [t.x, t.y, t.z]
    if not np.isfinite(m).all():
        raise ValueError('Nonfinite transform')
    return m


@dataclass
class Track:
    ident: int
    point: np.ndarray
    last: float
    votes: Counter = field(default_factory=Counter)
    hits: int = 0
    source: str = 'approx'

    @property
    def kind(self):
        return self.votes.most_common(1)[0][0]

    def confirmed(self, minimum):
        return self.hits >= minimum and self.votes[self.kind] / self.hits >= 0.75


class Tracker:
    def __init__(self, gate=0.25, min_hits=4):
        if not 0.05 <= gate <= 1.0 or min_hits < 2:
            raise ValueError('Invalid tracking parameters')
        self.gate, self.min_hits = gate, min_hits
        self.tracks, self.next_id = [], 0
        self.last_stamp = -1.0

    def update(self, observations, stamp):
        if stamp <= self.last_stamp:
            return  # Duplicate/old batches cannot create extra confirmation hits.
        self.last_stamp = stamp
        self.tracks = [t for t in self.tracks
                       if t.confirmed(self.min_hits) or stamp - t.last < 2.0]
        edges = []
        for oi, (kind, point, source) in enumerate(observations):
            if not np.isfinite(point).all():
                continue
            for ti, t in enumerate(self.tracks):
                # Do not merge established different colour types merely due to proximity.
                if t.confirmed(self.min_hits) and t.kind != kind:
                    continue
                distance = float(np.linalg.norm(t.point[:2] - point[:2]))
                if distance < self.gate:
                    edges.append((distance, oi, ti))
        assignments, used_tracks = {}, set()
        for _, oi, ti in sorted(edges):
            if oi not in assignments and ti not in used_tracks:
                assignments[oi] = self.tracks[ti]
                used_tracks.add(ti)
        # Observations with a plausible but already-used track are ambiguous, not new IDs.
        possible = {oi for _, oi, _ in edges}
        for oi, (kind, point, source) in enumerate(observations):
            if not np.isfinite(point).all():
                continue
            t = assignments.get(oi)
            if t is None:
                if oi in possible or len(self.tracks) >= 200:
                    continue
                t = Track(self.next_id, np.array(point, dtype=float), stamp)
                self.next_id += 1
                self.tracks.append(t)
            if source == 'lidar' and t.source != 'lidar':
                t.point = np.array(point, dtype=float)
            elif not (t.source == 'lidar' and source != 'lidar'):
                alpha = 1.0 / min(t.hits + 1, 20)
                t.point += alpha * (point - t.point)
            if source == 'lidar':
                t.source = 'lidar'
            t.votes[kind] += 1
            t.hits += 1
            t.last = stamp


def lidar_match(scan, scan_to_camera, k, distortion, box, diameter, max_range):
    """Project scan clusters into a detected body; return a measured scan-frame centre.

    Known-radius circle fit; no range from a background wall or an ambiguous cluster.
    A single scan pose is used: per-beam motion deskew is NOT implemented here.
    """
    import cv2
    if diameter <= 0 or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        return None
    raw = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(raw)) * scan.angle_increment
    valid = np.isfinite(raw) & (raw >= max(scan.range_min, 0.08)) & (raw <= min(scan.range_max, max_range))
    indices = np.flatnonzero(valid)
    if len(indices) < 3:
        return None
    pts = np.column_stack((raw[indices] * np.cos(angles[indices]),
                           raw[indices] * np.sin(angles[indices]), np.zeros(len(indices)),
                           np.ones(len(indices))))
    camera = (scan_to_camera @ pts.T).T[:, :3]
    forward = camera[:, 2] > 0.05
    indices, pts, camera = indices[forward], pts[forward], camera[forward]
    if len(indices) < 3:
        return None
    uv = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), k, distortion)[0].reshape(-1, 2)
    x, y, w, h = box
    inside = (uv[:, 0] >= x) & (uv[:, 0] <= x+w) & (uv[:, 1] >= y-2) & (uv[:, 1] <= y+h+2)
    groups, run = [], []
    for j in range(len(indices)):
        if not inside[j]:
            if run:
                groups.append(run)
                run = []
            continue
        if run and (indices[j] != indices[run[-1]]+1 or abs(raw[indices[j]]-raw[indices[run[-1]]]) > diameter):
            groups.append(run)
            run = []
        run.append(j)
    if run:
        groups.append(run)
    # A front-facing cylinder can straddle beam 359/0 on the real 360-degree scan.
    if (len(groups) > 1 and abs(abs(scan.angle_increment)*len(raw)-2*math.pi) < .05
            and indices[groups[0][0]] == 0 and indices[groups[-1][-1]] == len(raw)-1
            and abs(raw[0]-raw[-1]) <= diameter):
        groups = [groups[-1]+groups[0]] + groups[1:-1]
    matches = []
    radius = diameter/2
    for group in groups:
        if len(group) < 3:
            continue
        depth = float(np.median(camera[group, 2]))
        if not 0.55*diameter <= w * depth/k[0, 0] <= 1.65*diameter:
            continue
        points = pts[group, :2]
        if np.linalg.norm(np.ptp(points, axis=0)) > diameter*1.4:
            continue
        surface = np.median(points, axis=0)
        length = np.linalg.norm(surface)
        if length < 0.01:
            continue
        centre = surface + surface/length*radius
        for _ in range(8):
            delta = centre-points
            distances = np.linalg.norm(delta, axis=1)
            if np.any(distances < 1e-6):
                break
            jacobian = delta / distances[:, None]
            step = np.linalg.lstsq(jacobian, distances-radius, rcond=None)[0]
            centre -= step
            if np.linalg.norm(step) < 1e-5:
                break
        residual = np.abs(np.linalg.norm(points-centre, axis=1)-radius)
        if (not np.isfinite(centre).all() or np.linalg.norm(centre-surface) > diameter or
                np.linalg.norm(centre) < length or np.max(residual) > max(.008, diameter*.12)):
            continue
        error = abs(float(np.median(uv[group, 0]))-(x+w/2))/max(w, 1)
        if error < .35:
            matches.append((error, np.array([centre[0], centre[1], 0.])))
    matches.sort(key=lambda v: v[0])
    if not matches or (len(matches) > 1 and matches[1][0]-matches[0][0] < .10):
        return None
    return matches[0][1]
