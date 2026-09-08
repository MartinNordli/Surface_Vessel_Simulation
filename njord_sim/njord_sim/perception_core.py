"""Testable baseline camera/lidar fusion, buoy tracking and gate selection.

HSV segmentation assumes conspicuous red/green buoys. It is a simulator baseline,
not a learned detector or an assertion of all-weather field performance.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class Blob:
    color: str
    box: tuple
    score: float


def detect_blobs(bgr, min_area=20):
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    masks = {
        "red": cv2.inRange(hsv, (0, 100, 50), (12, 255, 255)) | cv2.inRange(hsv, (165, 100, 50), (179, 255, 255)),
        "green": cv2.inRange(hsv, (35, 90, 40), (90, 255, 255)),
    }
    output = []
    for color, mask in masks.items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if area >= min_area and 0.2 <= w/h <= 4.0 and area/(w*h) >= 0.35:
                output.append(Blob(color, (x, y, x+w, y+h), min(1., area/(w*h))))
    return output


def associate_lidar(blobs, optical_points, world_points, intrinsic, depth_tolerance=1.0):
    """Nearest coherent depth cluster inside each image blob, with >=2 returns."""
    camera = np.asarray(optical_points).reshape(-1, 3)
    world = np.asarray(world_points).reshape(-1, 3)
    valid = np.isfinite(camera).all(axis=1) & np.isfinite(world).all(axis=1) & (camera[:, 2] > 0.1)
    camera, world = camera[valid], world[valid]
    if not len(camera):
        return []
    projection = camera @ np.asarray(intrinsic).reshape(3, 3).T
    pixels = projection[:, :2] / projection[:, 2, None]
    result = []
    for blob in blobs:
        x0, y0, x1, y1 = blob.box
        inside = (pixels[:, 0] >= x0)&(pixels[:, 0] <= x1)&(pixels[:, 1] >= y0)&(pixels[:, 1] <= y1)
        indices = np.flatnonzero(inside)
        if len(indices) < 2:
            continue
        depth = np.percentile(camera[indices, 2], 20)
        indices = indices[np.abs(camera[indices, 2]-depth) <= depth_tolerance]
        if len(indices) >= 2:
            result.append((blob.color, np.median(world[indices], axis=0), blob.score))
    return result


class BuoyTracker:
    def __init__(self, distance=1.5, ttl=2.0, minimum_observations=3):
        self.distance, self.ttl, self.minimum = distance, ttl, minimum_observations
        self.tracks = []
        self.next_id = 0

    def update(self, observations, stamp):
        self.tracks = [t for t in self.tracks if 0 <= stamp-t["stamp"] <= self.ttl]
        used = set()
        for color, position, score in observations:
            candidates = [(np.linalg.norm(t["position"]-position), i) for i, t in enumerate(self.tracks)
                          if t["color"] == color and i not in used]
            distance, index = min(candidates, default=(math.inf, -1))
            if distance <= self.distance:
                t = self.tracks[index]
                if stamp > t["stamp"]:
                    t["count"] += 1
                t["position"] = 0.6*t["position"] + 0.4*np.asarray(position)
                t["stamp"], t["score"] = stamp, score
            else:
                t = dict(id=str(self.next_id), color=color, position=np.asarray(position),
                         score=score, stamp=stamp, count=1)
                self.next_id += 1
                self.tracks.append(t)
                index = len(self.tracks)-1
            used.add(index)
        # Only tracks observed in this image are returned: never re-stamp old tracks.
        return [self.tracks[i].copy() for i in used if self.tracks[i]["count"] >= self.minimum]


@dataclass
class Gate:
    center: np.ndarray
    forward: np.ndarray
    red: np.ndarray
    green: np.ndarray


def choose_gate(detections, position, heading=0., passed=(), min_width=8., max_width=30., ambiguity_m=3.):
    forward = np.array([math.cos(heading), math.sin(heading)])
    left = np.array([-forward[1], forward[0]])
    reds = [np.asarray(p)[:2] for color, p in detections if color == "red"]
    greens = [np.asarray(p)[:2] for color, p in detections if color == "green"]
    candidates = []
    for red in reds:
        for green in greens:
            across = red-green
            width = np.linalg.norm(across)
            if not min_width <= width <= max_width or across@left < width*0.7:
                continue
            normal = np.array([across[1], -across[0]])/width
            center = (red+green)/2
            relative = center-np.asarray(position)
            if relative@forward < 0.0 or abs(relative@left) > max(10., relative@forward):
                continue
            if any(np.linalg.norm(center-p) < min_width/2 for p in passed):
                continue
            # Same-color pairings across adjacent gates are suppressed by near-transverse alignment.
            if abs(across@forward) > width*0.45:
                continue
            candidates.append((float(np.linalg.norm(relative)), Gate(center, normal, red, green)))
    candidates.sort(key=lambda item: item[0])
    if not candidates:
        return None
    if len(candidates) > 1 and candidates[1][0]-candidates[0][0] < ambiguity_m:
        return None
    return candidates[0][1]
