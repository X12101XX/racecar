"""Robot-centric LiDAR panel for simulation videos.

Only sensor samples and the mission's fused tracks are drawn here; world truth
is deliberately not an input.
"""
from __future__ import annotations

import math
from typing import Any, Optional, Sequence, Tuple

from PIL import Image, ImageDraw

TRACK_COLORS = {
    "blue": (90, 170, 255),
    "red": (255, 105, 95),
    "unknown": (205, 212, 218),
}


def draw_lidar_view(scan: Optional[Any], pose: Optional[Sequence[float]],
                    tracks: Sequence[Any] = (), size: Tuple[int, int] = (320, 240),
                    max_range: float = 4.0, car_length: float = 0.42,
                    car_width: float = 0.30, title: str = "LIDAR / BODY FRAME") -> Image.Image:
    """Draw a body-frame scan with fused tracks and the vehicle footprint."""
    width, height = size
    img = Image.new("RGB", size, (9, 16, 23))
    draw = ImageDraw.Draw(img)
    cx, cy = width / 2.0, height * 0.58
    scale = max(1.0, min(width, height) * 0.42 / max(max_range, 1e-3))

    # Range rings and axes.
    for ring in range(1, int(max_range) + 1):
        radius = ring * scale
        draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius),
                     outline=(35, 58, 72), width=1)
        draw.text((cx + radius + 2, cy - 11), f"{ring}m", fill=(92, 120, 136))
    draw.line((cx, 8, cx, height - 8), fill=(31, 52, 64), width=1)
    draw.line((8, cy, width - 8, cy), fill=(31, 52, 64), width=1)
    draw.text((width - 24, cy + 3), "+x", fill=(100, 130, 145))
    draw.text((cx + 3, 8), "+y", fill=(100, 130, 145))

    # Raw scan points (sensor samples only).
    point_count = 0
    if scan is not None:
        try:
            angles = scan.angles
            ranges = scan.ranges
            for angle, distance in zip(angles, ranges):
                distance = float(distance)
                if not math.isfinite(distance) or distance <= 0.05 or distance > max_range:
                    continue
                # max_range circle is the panel edge; clamp only for drawing.
                x = cx + distance * math.cos(float(angle)) * scale
                y = cy - distance * math.sin(float(angle)) * scale
                if 0 <= x < width and 0 <= y < height:
                    color = (255, 198, 92) if distance < 1.2 else (135, 182, 204)
                    draw.point((x, y), fill=color)
                    point_count += 1
        except (TypeError, ValueError):
            pass

    # Mission-fused tracks, transformed into the estimated body frame.
    if pose is not None and len(pose) >= 3 and tracks:
        c, s = math.cos(float(pose[2])), math.sin(float(pose[2]))
        for track in tracks:
            try:
                dx, dy = float(track.x) - float(pose[0]), float(track.y) - float(pose[1])
                bx, by = c * dx + s * dy, -s * dx + c * dy
                if math.hypot(bx, by) > max_range:
                    continue
                x, y = cx + bx * scale, cy - by * scale
                radius = max(2.0, float(getattr(track, "radius", 0.11)) * scale)
                color = TRACK_COLORS.get(getattr(track, "color", "unknown"),
                                         TRACK_COLORS["unknown"])
                draw.ellipse((x - radius, y - radius, x + radius, y + radius),
                             outline=color, width=2)
            except (AttributeError, TypeError, ValueError):
                continue

    # Vehicle footprint and heading.
    hl, hw = car_length * scale / 2.0, car_width * scale / 2.0
    draw.rectangle((cx - hl, cy - hw, cx + hl, cy + hw),
                   outline=(120, 235, 205), width=2)
    draw.line((cx, cy, cx + hl * 1.8, cy), fill=(255, 225, 90), width=2)
    draw.polygon([(cx + hl * 1.8, cy), (cx + hl * 1.35, cy - 4),
                  (cx + hl * 1.35, cy + 4)], fill=(255, 225, 90))

    draw.text((6, 5), title, fill=(205, 225, 235))
    draw.text((6, height - 15), f"{point_count} pts  tracks={len(tracks)}",
              fill=(120, 155, 170))
    return img
