"""Saved maps for the kachaka-api map RPCs.

A map is a Nav2 map pair in the maps folder (repository `maps/`, mounted at
`/maps` in the ROS container): `<id>.yaml` plus its trinary `<id>.pgm`, as
`map_saver_cli` writes them. The yaml may carry two extra keys that Nav2
ignores: `name` (shown by GetMapList) and `charger_pose` ([x, y, yaw] in the
map frame, the SwitchMap default initial pose).

An exported map is a zip of the two files. Plain Python (numpy, Pillow, PyYAML),
no ROS and no Isaac Sim.
"""

from __future__ import annotations

import io
import re
import zipfile
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

# Grid values: 0 free, 100 occupied, -1 unknown (as in nav_msgs/OccupancyGrid).
FREE, OCCUPIED, UNKNOWN = 0, 100, -1
# PNG map gray levels the real robot uses (kachaka_grpc_ros2_bridge's mapping_component).
PNG_OCCUPIED, PNG_FREE, PNG_UNKNOWN = 174, 253, 234
# map_saver_cli's trinary pgm values and thresholds.
PGM_OCCUPIED, PGM_FREE, PGM_UNKNOWN = 0, 254, 205
OCCUPIED_THRESH, FREE_THRESH = 0.65, 0.196  # PGM_UNKNOWN reads as 0.19607: not free


class MapError(Exception):
    pass


class MapFile:
    """One map: grid rows run from the origin (min y) up, as in OccupancyGrid."""

    def __init__(self, grid: np.ndarray, resolution: float, origin: tuple[float, float, float],
                 name: str = "", charger_pose: tuple[float, float, float] | None = None):
        self.grid = grid
        self.resolution = resolution
        self.origin = origin
        self.name = name
        self.charger_pose = charger_pose

    # --- conversions ------------------------------------------------------------
    @classmethod
    def from_occupancy_grid(cls, data, width: int, height: int, resolution: float,
                            origin: tuple[float, float, float], **kw) -> MapFile:
        grid = np.asarray(data, np.int8).reshape(height, width).astype(np.int16)
        out = np.full(grid.shape, UNKNOWN, np.int16)
        out[(grid >= 0) & (grid < OCCUPIED_THRESH * 100)] = FREE
        out[grid >= OCCUPIED_THRESH * 100] = OCCUPIED
        return cls(out, resolution, origin, **kw)

    def png(self) -> bytes:
        """Kachaka's PNG map (top row = max y)."""
        img = np.full(self.grid.shape, PNG_UNKNOWN, np.uint8)
        img[self.grid == FREE] = PNG_FREE
        img[self.grid == OCCUPIED] = PNG_OCCUPIED
        out = io.BytesIO()
        Image.fromarray(img[::-1]).save(out, format="PNG")
        return out.getvalue()

    @classmethod
    def from_image(cls, data: bytes, resolution: float, **kw) -> MapFile:
        """A PNG or other image, top row = max y, origin at the bottom-left
        corner. Kachaka's own PNG palette is read as is; any other image by
        brightness: dark is occupied, light free, mid gray unknown."""
        img = Image.open(io.BytesIO(data))
        if img.mode in ("RGBA", "LA", "PA") or (img.mode == "P" and "transparency" in img.info):
            rgba = np.asarray(img.convert("RGBA"))
            gray = np.asarray(img.convert("L")).copy()
            gray[rgba[..., 3] < 128] = PNG_UNKNOWN  # transparent: unknown
        else:
            gray = np.asarray(img.convert("L"))
        grid = np.full(gray.shape, UNKNOWN, np.int16)
        if set(np.unique(gray).tolist()) <= {PNG_OCCUPIED, PNG_FREE, PNG_UNKNOWN}:
            grid[gray == PNG_FREE] = FREE
            grid[gray == PNG_OCCUPIED] = OCCUPIED
        else:
            grid[gray < 100] = OCCUPIED
            grid[gray > 200] = FREE
        return cls(grid[::-1].copy(), resolution, (0.0, 0.0, 0.0), **kw)

    # --- files -----------------------------------------------------------------
    def files(self, map_id: str) -> dict[str, bytes]:
        """{filename: content} of the Nav2 map pair."""
        pgm = np.full(self.grid.shape, PGM_UNKNOWN, np.uint8)
        pgm[self.grid == FREE] = PGM_FREE
        pgm[self.grid == OCCUPIED] = PGM_OCCUPIED
        out = io.BytesIO()
        Image.fromarray(pgm[::-1]).save(out, format="PPM")  # 8-bit gray -> binary pgm (P5)
        meta = {"image": f"{map_id}.pgm", "mode": "trinary", "resolution": float(self.resolution),
                "origin": [float(v) for v in self.origin], "negate": 0,
                "occupied_thresh": OCCUPIED_THRESH, "free_thresh": FREE_THRESH,
                "name": self.name or map_id}
        if self.charger_pose is not None:
            meta["charger_pose"] = [float(v) for v in self.charger_pose]
        return {f"{map_id}.yaml": yaml.safe_dump(meta, sort_keys=False).encode(), f"{map_id}.pgm": out.getvalue()}

    @classmethod
    def from_files(cls, meta: dict, image: bytes) -> MapFile:
        img = np.asarray(Image.open(io.BytesIO(image)).convert("L"), np.float32)[::-1]
        p = img / 255.0 if meta.get("negate") else (255.0 - img) / 255.0
        grid = np.full(img.shape, UNKNOWN, np.int16)
        grid[p > float(meta.get("occupied_thresh", OCCUPIED_THRESH))] = OCCUPIED
        grid[p < float(meta.get("free_thresh", FREE_THRESH))] = FREE
        origin = tuple(float(v) for v in meta.get("origin", [0, 0, 0])[:3])
        charger = meta.get("charger_pose")
        return cls(grid, float(meta["resolution"]), origin, name=str(meta.get("name", "")),
                   charger_pose=tuple(float(v) for v in charger) if charger else None)

    def zip(self, map_id: str) -> bytes:
        out = io.BytesIO()
        with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
            for name, content in self.files(map_id).items():
                z.writestr(name, content)
        return out.getvalue()

    @classmethod
    def from_zip(cls, data: bytes) -> MapFile:
        try:
            z = zipfile.ZipFile(io.BytesIO(data))
            yamls = [n for n in z.namelist() if n.endswith(".yaml")]
            if len(yamls) != 1:
                raise MapError("the archive must hold one map yaml")
            meta = yaml.safe_load(z.read(yamls[0]))
            image = z.read(str(Path(yamls[0]).parent / meta["image"]).removeprefix("./"))
        except (zipfile.BadZipFile, KeyError, yaml.YAMLError) as e:
            raise MapError(f"not a map archive: {e}") from e
        return cls.from_files(meta, image)


class MapStore:
    """The maps folder."""

    def __init__(self, folder: Path, reserved: tuple[str, ...] = ()):
        self.folder = Path(folder)
        self.reserved = reserved  # ids save() never gives

    def ids(self) -> list[str]:
        return sorted(p.stem for p in self.folder.glob("*.yaml"))

    def meta(self, map_id: str) -> dict:
        path = self._yaml(map_id)
        try:
            return yaml.safe_load(path.read_text()) or {}
        except (OSError, yaml.YAMLError) as e:
            raise MapError(f"cannot read {path}: {e}") from e

    def load(self, map_id: str) -> MapFile:
        meta = self.meta(map_id)
        image = Path(meta.get("image", ""))
        path = image if image.is_absolute() else self.folder / image
        try:
            return MapFile.from_files(meta, path.read_bytes())
        except (OSError, KeyError, ValueError) as e:
            raise MapError(f"cannot read map {map_id}: {e}") from e

    def save(self, m: MapFile, name: str = "") -> str:
        """Write a new map; returns its id, made from the name."""
        base = re.sub(r"[^A-Za-z0-9_-]+", "_", name or m.name or "map").strip("_") or "map"
        map_id, n = base, 1
        while map_id in self.reserved or (self.folder / f"{map_id}.yaml").exists():
            n += 1
            map_id = f"{base}_{n}"
        self.folder.mkdir(parents=True, exist_ok=True)
        for fname, content in m.files(map_id).items():
            (self.folder / fname).write_bytes(content)
        return map_id

    def _yaml(self, map_id: str) -> Path:
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", map_id) or map_id.startswith("."):
            raise MapError(f"bad map id {map_id!r}")
        path = self.folder / f"{map_id}.yaml"
        if not path.exists():
            raise MapError(f"no map {map_id!r} in {self.folder}")
        return path
