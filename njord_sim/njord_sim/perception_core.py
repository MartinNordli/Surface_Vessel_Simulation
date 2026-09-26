"""Testable baseline camera/lidar fusion, buoy tracking and gate selection.

HSV segmentation assumes conspicuous red/green buoys. It is a simulator baseline,
not a learned detector or an assertion of all-weather field performance.

Pipeline, as used by ``perception_node`` for each camera image:

1. ``detect_blobs`` finds red and green regions in the image (pixels).
2. ``associate_lidar`` projects lidar points into the image and turns each
   blob with enough lidar support into a 3D position in the ``map`` frame.
3. ``BuoyTracker`` merges these per-image observations over time and only
   reports buoys that were seen several times.

``choose_gate`` is used downstream by ``mission_node`` to pick the red/green pair
the vessel should pass through next. Everything here is pure Python/NumPy
(OpenCV only for ``detect_blobs``) so it can be tested without ROS or Gazebo.
Only sensor-derived positions enter these functions; scenario ground truth is
never used.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class Blob:
    """One colour region found in an image.

    Attributes:
        color: ``"red"`` or ``"green"``.
        box: pixel bounding box ``(x0, y0, x1, y1)``; x to the right, y down,
            with ``x1 = x0 + width`` and ``y1 = y0 + height``.
        score: fill ratio of the contour in its bounding box, in [0.35, 1].
            A shape cue, not a calibrated probability.
    """
    color: str
    box: tuple
    score: float


def detect_blobs(bgr, min_area=20):
    """Find red and green buoy candidates in a BGR image by HSV thresholding.

    Thresholds use OpenCV's HSV scale (hue 0-179, saturation and value 0-255):

    * red: hue 0-12 or 165-179 (red wraps around hue 0), S >= 100, V >= 50;
    * green: hue 35-90, S >= 90, V >= 40.

    Each outer contour of a colour mask is kept only if it passes simple shape
    filters that reject noise, thin streaks and ragged regions:

    * contour area >= ``min_area`` pixels;
    * bounding-box aspect ratio width/height between 0.2 and 4.0;
    * fill ratio (contour area / bounding-box area) >= 0.35.

    Args:
        bgr: (H, W, 3) uint8 image in BGR channel order.
        min_area: minimum contour area in pixels.

    Returns:
        List of ``Blob``; the score is the fill ratio capped at 1.
    """
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    masks = {
        # Red sits at both ends of the hue circle, so two ranges are combined.
        "red": (cv2.inRange(hsv, (0, 100, 50), (12, 255, 255))
                | cv2.inRange(hsv, (165, 100, 50), (179, 255, 255))),
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
    """Nearest coherent depth cluster inside each image blob, with >=2 returns.

    The same lidar points are passed twice, row by row: once in the camera
    optical frame (x right, y down, z forward) to project them into the image,
    and once in the ``map`` frame to report the buoy position.

    For each blob:

    1. Keep points whose pinhole projection ``K @ p`` lands inside the blob's
       bounding box (edges inclusive). At least 2 are required.
    2. Take the 20th percentile of their optical depth (z) as the depth of the
       object. A low percentile favours the nearest surface (the buoy) over
       background returns seen through the box, while one spurious very close
       point cannot pull it all the way forward.
    3. Keep only points within ``depth_tolerance`` metres of that depth.
    4. If at least 2 remain, the buoy position is their per-axis median in map.

    Points that are non-finite in either frame, or less than 0.1 m in front
    of the camera, are ignored. Lens distortion is not modelled, so the image
    must be undistorted (the node checks this).

    Args:
        blobs: iterable of ``Blob`` from ``detect_blobs``.
        optical_points: (N, 3) lidar points in the camera optical frame [m].
        world_points: (N, 3) the same points in the ``map`` frame [m].
        intrinsic: 3x3 camera matrix K, or the 9 row-major values of
            ``CameraInfo.k`` [pixels].
        depth_tolerance: half-width of the accepted depth band [m].

    Returns:
        List of ``(color, position, score)``, where ``position`` is a (3,)
        map-frame array [m] and ``score`` is the blob score. Blobs without
        enough lidar support are dropped.
    """
    camera = np.asarray(optical_points).reshape(-1, 3)
    world = np.asarray(world_points).reshape(-1, 3)
    # z > 0.1 m: only points in front of the camera can project into the image.
    valid = (np.isfinite(camera).all(axis=1) & np.isfinite(world).all(axis=1)
             & (camera[:, 2] > 0.1))
    camera, world = camera[valid], world[valid]
    if not len(camera):
        return []
    # Pinhole projection: [u*z, v*z, z] = K @ [x, y, z]; divide by z for pixels.
    projection = camera @ np.asarray(intrinsic).reshape(3, 3).T
    pixels = projection[:, :2] / projection[:, 2, None]
    result = []
    for blob in blobs:
        x0, y0, x1, y1 = blob.box
        inside = ((pixels[:, 0] >= x0)&(pixels[:, 0] <= x1)
                  & (pixels[:, 1] >= y0)&(pixels[:, 1] <= y1))
        indices = np.flatnonzero(inside)
        if len(indices) < 2:
            continue
        depth = np.percentile(camera[indices, 2], 20)
        indices = indices[np.abs(camera[indices, 2]-depth) <= depth_tolerance]
        if len(indices) >= 2:
            result.append((blob.color, np.median(world[indices], axis=0), blob.score))
    return result


class BuoyTracker:
    """Associate per-image buoy observations over time (one tracker per camera).

    A track is a dict with ``id`` (string, unique per tracker), ``color``,
    ``position`` (map frame [m], same dimension as the observations),
    ``score`` (latest observation score), ``stamp`` (acquisition time [s] of
    the latest observation) and ``count`` (number of distinct image stamps in
    which it was seen).

    Args:
        distance: association gate [m]. An observation updates the nearest
            same-colour track within this Euclidean distance; otherwise it
            starts a new track.
        ttl: a track not observed for more than ``ttl`` seconds of image time
            is dropped.
        minimum_observations: a track is reported only after being seen in at
            least this many images, which suppresses one-off false positives.
    """

    def __init__(self, distance=1.5, ttl=2.0, minimum_observations=3):
        self.distance, self.ttl, self.minimum = distance, ttl, minimum_observations
        self.tracks = []
        self.next_id = 0

    def update(self, observations, stamp):
        """Integrate the observations of one image and return confirmed tracks.

        Args:
            observations: list of ``(color, position, score)`` from
                ``associate_lidar``; positions in the map frame [m].
            stamp: acquisition time of the image [s, simulation time].

        Returns:
            Copies of the tracks updated by *this* image that have reached
            ``minimum_observations``. Tracks that were not seen in this image
            are not returned, even if still alive.
        """
        # Expire old tracks. Tracks stamped after ``stamp`` (time went
        # backwards) are dropped as well.
        self.tracks = [t for t in self.tracks if 0 <= stamp-t["stamp"] <= self.ttl]
        used = set()
        for color, position, score in observations:
            # Greedy nearest-neighbour association in observation order; each
            # track can absorb at most one observation per image.
            candidates = [(np.linalg.norm(t["position"]-position), i)
                          for i, t in enumerate(self.tracks)
                          if t["color"] == color and i not in used]
            distance, index = min(candidates, default=(math.inf, -1))
            if distance <= self.distance:
                t = self.tracks[index]
                # Count distinct images only, so re-processing the same stamp
                # cannot confirm a track.
                if stamp > t["stamp"]:
                    t["count"] += 1
                # Exponential smoothing: 60 % previous estimate, 40 % new one.
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
    """A red/green buoy pair chosen as the next gate (2D, map frame).

    Attributes:
        center: midpoint between the two buoys (x, y) [m].
        forward: unit vector perpendicular to the red-green line, pointing in
            the direction of passage (the side the vessel is heading towards).
        red: red buoy (x, y) [m]; lies to the left of the passage direction.
        green: green buoy (x, y) [m]; lies to the right.
    """
    center: np.ndarray
    forward: np.ndarray
    red: np.ndarray
    green: np.ndarray


def choose_gate(detections, position, heading=0., passed=(), min_width=8., max_width=30.,
                ambiguity_m=3.):
    """Pick the nearest plausible red/green gate ahead of the vessel.

    Every red/green pair is tested against these geometric filters:

    * width (buoy separation) within ``[min_width, max_width]`` metres;
    * seen from the vessel heading, red must be on the left and green on the
      right: the red-minus-green vector must point within about 46 degrees
      of the vessel's left axis (component >= 0.7 * width);
    * the gate centre must be ahead of the vessel, and its lateral offset may
      not exceed ``max(10 m, distance ahead)`` (a 45-degree cone that is at
      least 10 m wide on each side close to the boat);
    * the centre must be more than ``min_width / 2`` from every gate in
      ``passed``, so gates already crossed are not chosen again;
    * the red-green line must be nearly transverse to the heading (its
      forward component at most 0.45 * width, about 27 degrees), which
      rejects diagonal pairings of buoys from two adjacent gates.

    The survivors are ranked by distance from the vessel to the gate centre.
    Ambiguity rule: if the second-nearest candidate is less than
    ``ambiguity_m`` further away than the nearest, no gate is returned,
    rather than guessing between two similar options.

    Args:
        detections: iterable of ``(color, position)`` with positions in the
            map frame; only x and y are used.
        position: vessel (x, y) in map [m], from the navigation estimate.
        heading: vessel yaw [rad], ENU, counter-clockwise from +x (east).
        passed: iterable of (x, y) centres of gates already passed [m].
        min_width, max_width: accepted gate width range [m].
        ambiguity_m: minimum distance margin between the best two candidates.

    Returns:
        A ``Gate``, or ``None`` if there is no candidate or the choice is
        ambiguous.
    """
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
            # ``across`` rotated 90 degrees clockwise: the passage direction.
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
