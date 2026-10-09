"""ROS-independent rectangle geometry and YAML persistence."""
from dataclasses import dataclass, asdict
from math import cos, sin, isfinite
from pathlib import Path
import os
import tempfile
import yaml


@dataclass
class Zone:
    name: str
    x: float
    y: float
    yaw: float
    width: float
    height: float

    def local(self, x, y):
        dx, dy = x - self.x, y - self.y
        c, s = cos(self.yaw), sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def world(self, x, y):
        c, s = cos(self.yaw), sin(self.yaw)
        return self.x + c * x - s * y, self.y + s * x + c * y

    def contains(self, x, y, margin=0.0):
        u, v = self.local(x, y)
        return (abs(u) <= self.width / 2 + margin + 1e-9 and
                abs(v) <= self.height / 2 + margin + 1e-9)


def load_zones(path, frame):
    path = Path(path).expanduser()
    if not path.exists():
        return []
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or data.get('frame_id') != frame:
        raise ValueError(f'YAML frame_id must be {frame!r}')
    rows = data.get('zones')
    if not isinstance(rows, list):
        raise ValueError('zones must be a list')
    zones, names = [], set()
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError('each zone must be a mapping')
        name = row['name']
        if (not isinstance(name, str) or not name or name in names or name.endswith('__size') or
                not all(c.isalnum() or c in '_-' for c in name)):
            raise ValueError('zone names must be unique and contain letters, numbers, _ or -')
        vals = {k: float(row[k]) for k in ('x', 'y', 'yaw', 'width', 'height')}
        if not all(isfinite(v) for v in vals.values()):
            raise ValueError('zone values must be finite')
        if vals['width'] < 0.1 or vals['height'] < 0.1:
            raise ValueError('width and height must be >= 0.1 m')
        zones.append(Zone(name=name, **vals))
        names.add(name)
    return zones


def save_zones(path, frame, zones):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {'frame_id': frame, 'zones': [asdict(z) for z in zones]}
    # Atomic replace: keep the previous complete file if writing fails.
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name + '.')
    try:
        with os.fdopen(fd, 'w') as f:
            yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
