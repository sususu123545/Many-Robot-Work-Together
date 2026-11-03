"""RGB-D color-object candidates for the independent G1 competition tasks.

This is deliberately sensor-only: it consumes an RGB image, metric depth, and
camera intrinsics. It never reads USD prim labels, object poses, or simulator
ground truth. Red/yellow are only color proposals; the judge's required order
and any medical-item classes must be supplied in the mission configuration.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Mapping

import numpy as np


# Optical camera coordinates are right/down/forward; robot coordinates are
# forward/left/up. These match the independent FrontRGBD sensor calibration.
CAMERA_POSITION_BASE = np.array([0.16, 0.0, 0.23], dtype=np.float64)
CAMERA_PITCH_RAD = np.deg2rad(15.0)


def optical_to_base() -> np.ndarray:
    c, s = np.cos(CAMERA_PITCH_RAD), np.sin(CAMERA_PITCH_RAD)
    level = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    pitch = np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
    return pitch @ level


@dataclass(frozen=True)
class ColorBand:
    """A simple HSV band; hue is degrees, saturation/value are in [0, 1]."""

    hue_min: float
    hue_max: float
    saturation_min: float = 0.48
    value_min: float = 0.16
    kind: str = "color_target"

    @classmethod
    def from_mapping(cls, data: Mapping[str, object]) -> "ColorBand":
        hue = data.get("hue_degrees")
        if not isinstance(hue, (list, tuple)) or len(hue) != 2:
            raise ValueError("each color band requires hue_degrees: [min, max]")
        lo, hi = float(hue[0]), float(hue[1])
        if not (0 <= lo <= 360 and 0 <= hi <= 360):
            raise ValueError("HSV hue values must be between 0 and 360 degrees")
        kind = str(data.get("kind", "color_target"))
        if kind not in {"wire_candidate", "supply_candidate", "kit_candidate", "color_target"}:
            raise ValueError(f"unsupported detection kind: {kind}")
        return cls(lo, hi, float(data.get("saturation_min", .48)),
                   float(data.get("value_min", .16)), kind)


@dataclass(frozen=True)
class Detection:
    label: str
    kind: str
    confidence: float
    position_base: tuple[float, float, float]
    timestamp: float
    frame_id: int
    pixel: tuple[int, int]
    area_px: int
    track_id: str | None = None


DEFAULT_COLOR_BANDS = {
    # Broad prototype bands for the independent scene. Tune from the actual
    # competition camera and lighting before using on the real robot.
    "red": ColorBand(345.0, 18.0, kind="wire_candidate"),
    "yellow": ColorBand(34.0, 76.0, kind="wire_candidate"),
}


def _rgb_to_hsv(rgb: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] < 3:
        raise ValueError("rgb must have shape [height, width, 3]")
    x = image[..., :3].astype(np.float32)
    if x.size and float(np.nanmax(x)) > 1.5:
        x /= 255.0
    x = np.clip(np.nan_to_num(x, nan=0.0), 0.0, 1.0)
    r, g, b = x[..., 0], x[..., 1], x[..., 2]
    mx, mn = np.max(x, axis=2), np.min(x, axis=2)
    delta = mx - mn
    hue = np.zeros_like(mx)
    nonzero = delta > 1e-6
    rmax = nonzero & (mx == r)
    gmax = nonzero & (mx == g)
    bmax = nonzero & (mx == b)
    hue[rmax] = (60.0 * ((g[rmax] - b[rmax]) / delta[rmax])) % 360.0
    hue[gmax] = 60.0 * ((b[gmax] - r[gmax]) / delta[gmax] + 2.0)
    hue[bmax] = 60.0 * ((r[bmax] - g[bmax]) / delta[bmax] + 4.0)
    sat = np.divide(delta, mx, out=np.zeros_like(mx), where=mx > 1e-6)
    return hue, sat, mx


def _components(mask: np.ndarray) -> list[np.ndarray]:
    """Return 8-connected pixel coordinates without requiring OpenCV."""
    try:
        import cv2
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask.astype(np.uint8), connectivity=8
        )
        return [np.column_stack(np.where(labels == i))
                for i in range(1, count)
                if stats[i, cv2.CC_STAT_AREA] > 0]
    except ImportError:
        pass

    # Sparse flood fill fallback for environments without OpenCV.
    remaining = set(map(tuple, np.argwhere(mask)))
    found = []
    while remaining:
        seed = remaining.pop()
        stack, pixels = [seed], [seed]
        while stack:
            y, x = stack.pop()
            for yy in range(max(0, y - 1), min(mask.shape[0], y + 2)):
                for xx in range(max(0, x - 1), min(mask.shape[1], x + 2)):
                    point = (yy, xx)
                    if point in remaining:
                        remaining.remove(point)
                        stack.append(point)
                        pixels.append(point)
        found.append(np.asarray(pixels, dtype=np.int32))
    return found


class RGBDColorDetector:
    """Detect configured color blobs and project their centers into G1 base."""

    def __init__(self, color_bands: Mapping[str, ColorBand] | None = None,
                 min_area_px: int = 7, min_confidence: float = .58):
        self.color_bands = dict(color_bands or DEFAULT_COLOR_BANDS)
        self.min_area_px = int(min_area_px)
        self.min_confidence = float(min_confidence)
        self._tracks: dict[str, dict[str, object]] = {}
        self._next_track = 1
        if self.min_area_px < 1 or not 0 <= self.min_confidence <= 1:
            raise ValueError("invalid detector thresholds")

    @classmethod
    def from_config(cls, data: Mapping[str, object]) -> "RGBDColorDetector":
        raw = data.get("categories", {})
        if not isinstance(raw, Mapping):
            raise ValueError("perception.categories must be a mapping")
        bands = {str(label): ColorBand.from_mapping(spec)
                 for label, spec in raw.items()}
        if not bands:
            bands = DEFAULT_COLOR_BANDS.copy()
        return cls(bands, int(data.get("min_area_px", 7)),
                   float(data.get("min_confidence", .58)))

    def detect(self, frame: Mapping[str, object], frame_id: int | None = None) -> list[Detection]:
        rgb = np.asarray(frame["rgb"])
        depth = np.asarray(frame["depth"], dtype=np.float32).squeeze()
        intrinsics = np.asarray(frame["K"], dtype=np.float64)
        if rgb.shape[:2] != depth.shape or intrinsics.shape != (3, 3):
            raise ValueError("RGB, depth, and 3x3 intrinsics must describe the same frame")
        stamp = float(frame.get("seconds", 0.0))
        fid = int(stamp * 10) if frame_id is None else int(frame_id)
        hue, sat, value = _rgb_to_hsv(rgb)
        detections: list[Detection] = []

        for label, band in self.color_bands.items():
            hue_mask = ((hue >= band.hue_min) | (hue <= band.hue_max)
                        if band.hue_min > band.hue_max
                        else (hue >= band.hue_min) & (hue <= band.hue_max))
            mask = hue_mask & (sat >= band.saturation_min) & (value >= band.value_min)
            for component in _components(mask):
                if len(component) < self.min_area_px:
                    continue
                ys, xs = component[:, 0], component[:, 1]
                ds = depth[ys, xs]
                valid = np.isfinite(ds) & (ds >= .2) & (ds <= 8.0)
                if int(valid.sum()) < max(3, self.min_area_px // 2):
                    continue
                ys, xs, ds = ys[valid], xs[valid], ds[valid]
                u, v, z = float(np.median(xs)), float(np.median(ys)), float(np.median(ds))
                optical = np.array([(u - intrinsics[0, 2]) * z / intrinsics[0, 0],
                                    (v - intrinsics[1, 2]) * z / intrinsics[1, 1], z])
                base = CAMERA_POSITION_BASE + optical_to_base() @ optical
                mean_sat = float(np.mean(sat[component[:, 0], component[:, 1]]))
                confidence = float(np.clip(.55 * mean_sat + .45 * min(1., len(component) / 36.), 0., 1.))
                if confidence < self.min_confidence:
                    continue
                detections.append(Detection(
                    label=str(label), kind=band.kind, confidence=confidence,
                    position_base=tuple(float(x) for x in base), timestamp=stamp,
                    frame_id=fid, pixel=(int(round(u)), int(round(v))), area_px=int(len(component))))

        detections.sort(key=lambda item: (item.position_base[0], -item.confidence))
        assigned: set[str] = set()
        tracked = []
        for item in detections:
            choices = []
            for track_id, track in self._tracks.items():
                if (track_id in assigned or track["label"] != item.label
                        or fid - int(track["frame_id"]) > 3):
                    continue
                distance = float(np.linalg.norm(
                    np.asarray(track["position"], dtype=np.float64)
                    - np.asarray(item.position_base, dtype=np.float64)))
                if distance < .45:
                    choices.append((distance, track_id))
            if choices:
                _, track_id = min(choices)
            else:
                track_id = f"color-{self._next_track:04d}"
                self._next_track += 1
            assigned.add(track_id)
            self._tracks[track_id] = {"label": item.label,
                                      "position": item.position_base,
                                      "frame_id": fid}
            tracked.append(replace(item, track_id=track_id))
        self._tracks = {key: value for key, value in self._tracks.items()
                        if fid - int(value["frame_id"]) <= 3}
        return tracked
