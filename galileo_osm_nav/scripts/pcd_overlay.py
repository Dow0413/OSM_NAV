"""Lightweight north-up PCD rasters for visual OSM alignment checks."""

from io import BytesIO
from pathlib import Path
import math
import threading

import numpy as np
from PIL import Image
import yaml


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
PCD_FIELDS = ("x", "y", "z", "intensity")


class PcdOverlay:
    def __init__(self, map_dir, sample_step=4, pixel_size_m=0.5):
        map_dir = Path(map_dir)
        manifest = yaml.safe_load((map_dir / "manifest.yaml").read_text(encoding="utf-8"))
        origin = manifest.get("enu_origin_lla")
        matrix = manifest.get("t_align")
        if not isinstance(origin, list) or len(origin) != 3:
            raise ValueError("manifest.yaml requires enu_origin_lla: [latitude, longitude, altitude]")
        if not isinstance(matrix, list) or len(matrix) != 16:
            raise ValueError("manifest.yaml requires a 4x4 row-major t_align")
        self.origin = [float(value) for value in origin]
        self.matrix = np.asarray(matrix, dtype=float).reshape(4, 4)
        if not np.allclose(self.matrix[3], [0, 0, 0, 1]):
            raise ValueError("t_align is not a homogeneous 4x4 transform")
        self.inverse_matrix = np.linalg.inv(self.matrix)
        self.map_path = map_dir / manifest["global_map_file"]
        header = {}
        with self.map_path.open("rb") as stream:
            while True:
                line = stream.readline()
                if not line:
                    raise ValueError("PCD header is incomplete")
                key, _, value = line.decode("ascii").strip().partition(" ")
                header[key] = value
                if key == "DATA":
                    data_offset = stream.tell()
                    break
        if (header.get("DATA") != "binary" or header.get("FIELDS", "").split() != list(PCD_FIELDS)
                or header.get("SIZE", "").split() != ["4"] * 4
                or header.get("TYPE", "").split() != ["F"] * 4
                or header.get("COUNT", "").split() != ["1"] * 4):
            raise ValueError("PCD overlay currently supports binary float32 x y z intensity maps")
        self.point_count = int(header["POINTS"])
        expected_size = data_offset + self.point_count * 16
        if self.map_path.stat().st_size < expected_size:
            raise ValueError("PCD point data is shorter than its header declares")
        self.points = np.memmap(self.map_path, dtype="<f4", mode="r", offset=data_offset,
                                shape=(self.point_count, 4))
        self.sample_step = max(1, int(sample_step))
        self.pixel_size_m = float(pixel_size_m)
        if self.pixel_size_m <= 0:
            raise ValueError("pixel_size_m must be positive")
        self._cache = {}
        self._cache_lock = threading.Lock()
        self._bounds = {frame: self._compute_bounds(frame) for frame in ("raw", "aligned", "inverse")}

    def _xy_z(self, frame):
        sample = self.points[::self.sample_step, :3]
        if frame == "raw":
            return sample[:, 0], sample[:, 1], sample[:, 2]
        matrix = self.matrix if frame == "aligned" else self.inverse_matrix
        xyz = sample @ matrix[:3, :3].T + matrix[:3, 3]
        return xyz[:, 0], xyz[:, 1], xyz[:, 2]

    def _compute_bounds(self, frame):
        x, y, _ = self._xy_z(frame)
        valid = np.isfinite(x) & np.isfinite(y)
        if not np.any(valid):
            raise ValueError("PCD has no finite XY points")
        return (float(np.min(x[valid]) - 2), float(np.min(y[valid]) - 2),
                float(np.max(x[valid]) + 2), float(np.max(y[valid]) + 2))

    def _to_lla(self, east, north):
        lat0, lon0, altitude = self.origin
        latitude = math.radians(lat0)
        sin_lat = math.sin(latitude)
        radius_prime = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)
        radius_meridian = WGS84_A * (1 - WGS84_E2) / (1 - WGS84_E2 * sin_lat ** 2) ** 1.5
        return [lat0 + math.degrees(north / (radius_meridian + altitude)),
                lon0 + math.degrees(east / ((radius_prime + altitude) * math.cos(latitude)))]

    def geographic_bounds(self, frame):
        xmin, ymin, xmax, ymax = self._bounds[frame]
        return [self._to_lla(xmin, ymin), self._to_lla(xmax, ymax)]

    def metadata(self):
        return {
            "available": True,
            "file": self.map_path.name,
            "point_count": self.point_count,
            "enu_origin_lla": self.origin,
            "t_align": self.matrix.ravel().tolist(),
            "bounds": {frame: self.geographic_bounds(frame) for frame in self._bounds},
            "frames": ["raw", "aligned", "inverse"],
            "height_modes": ["all", "high"],
            "sample_step": self.sample_step,
            "pixel_size_m": self.pixel_size_m,
        }

    def png(self, frame, height_mode):
        if frame not in self._bounds or height_mode not in ("all", "high"):
            raise ValueError("frame must be raw/aligned/inverse and height must be all/high")
        key = (frame, height_mode)
        with self._cache_lock:
            if key in self._cache:
                return self._cache[key]
            x, y, z = self._xy_z(frame)
            valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            if height_mode == "high":
                valid &= z >= 3.0
            xmin, ymin, xmax, ymax = self._bounds[frame]
            width = min(2048, max(1, math.ceil((xmax - xmin) / self.pixel_size_m)))
            height = min(2048, max(1, math.ceil((ymax - ymin) / self.pixel_size_m)))
            density, _, _ = np.histogram2d(y[valid], x[valid], bins=(height, width),
                                           range=((ymin, ymax), (xmin, xmax)))
            density = np.flipud(density)
            rgba = np.zeros((height, width, 4), dtype=np.uint8)
            rgba[:, :, :3] = (0, 210, 235) if height_mode == "high" else (59, 112, 166)
            rgba[:, :, 3] = np.minimum(210, np.log1p(density) * 80).astype(np.uint8)
            output = BytesIO()
            Image.fromarray(rgba, "RGBA").save(output, format="PNG", optimize=True)
            result = output.getvalue()
            self._cache[key] = result
            return result
