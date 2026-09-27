"""Self-return rejection against indexed visible triangles in the body frame.

Only distance to a real surface counts: neither a hull bounding box nor the
interior of a convex vessel envelope is treated as a measured self return.
"""
import itertools
import json
import math
from pathlib import Path
import numpy as np


class SelfGeometry:
    def __init__(self, artifact, margin, frame='wamv/base_link'):
        if artifact.get('version') != 1 or artifact.get('frame') != frame:
            raise ValueError('self geometry version/frame mismatch')
        if not math.isfinite(margin) or margin < 0:
            raise ValueError('invalid self-filter margin')
        triangles = []
        for surface in artifact['surfaces']:
            vertices = np.asarray(surface['vertices'], dtype=float)
            raw_faces = np.asarray(surface['faces'])
            if (vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all()
                    or raw_faces.ndim != 2 or raw_faces.shape[1] != 3
                    or not np.issubdtype(raw_faces.dtype, np.integer)
                    or (raw_faces < 0).any() or (raw_faces >= len(vertices)).any()):
                raise ValueError('invalid indexed self geometry')
            triangles.extend(vertices[raw_faces])
        self.triangles = np.asarray(triangles)
        if not len(triangles):
            raise ValueError('self geometry has no visible surfaces')
        edges1 = self.triangles[:, 1]-self.triangles[:, 0]
        edges2 = self.triangles[:, 2]-self.triangles[:, 0]
        if (np.linalg.norm(np.cross(edges1, edges2), axis=1) <= 1e-12).any():
            raise ValueError('degenerate self geometry triangle')
        self.margin = margin
        self.cell_size = max(.25, 2*margin)
        self.index = {}
        for i, triangle in enumerate(self.triangles):
            low = np.floor((triangle.min(axis=0)-margin)/self.cell_size).astype(int)
            high = np.floor((triangle.max(axis=0)+margin)/self.cell_size).astype(int)
            if np.prod(high-low+1) > 1000000:
                raise ValueError('self geometry exceeds spatial index bounds')
            for key in itertools.product(*(range(a, b+1) for a, b in zip(low, high))):
                self.index.setdefault(key, []).append(i)

    @classmethod
    def load(cls, path, margin, frame='wamv/base_link'):
        # Missing or invalid artifacts fail closed: there is no generic hull fallback.
        return cls(json.loads(Path(path).read_text()), margin, frame)

    @classmethod
    def load_frames(cls, path, margin, frame='wamv/base_link'):
        if not path or not Path(path).is_file():
            raise ValueError(f'required self geometry artifact is missing: {path!r}')
        artifact = json.loads(Path(path).read_text())
        if artifact.get('version') != 1 or artifact.get('frame') != frame:
            raise ValueError('self geometry version/frame mismatch')
        groups = {}
        for surface in artifact['surfaces']:
            surface_frame = surface.get('frame', frame)
            if not isinstance(surface_frame, str) or not surface_frame:
                raise ValueError('missing self geometry surface frame')
            groups.setdefault(surface_frame, []).append(surface)
        if not groups:
            raise ValueError('self geometry has no visible surfaces')
        return {key: cls({'version': 1, 'frame': key, 'surfaces': surfaces}, margin, key)
                for key, surfaces in groups.items()}

    def contains_returns(self, points):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        result = np.zeros(len(points), dtype=bool)
        buckets = {}
        for i, point in enumerate(points):
            if np.isfinite(point).all():
                buckets.setdefault(tuple(np.floor(point/self.cell_size).astype(int)), []).append(i)
        for key, rows in buckets.items():
            candidates = self.index.get(key)
            if not candidates:
                continue
            p = points[rows, None, :]
            t = self.triangles[candidates]
            a, b, c = t[:, 0], t[:, 1], t[:, 2]
            ab, ac = b-a, c-a
            n = np.cross(ab, ac)
            n2 = np.sum(n*n, axis=-1)
            signed = np.sum((p-a)*n, axis=-1)
            projected = p-signed[..., None]*n/n2[:, None]
            v = projected-a
            d00, d01, d11 = (np.sum(ab*ab, axis=-1), np.sum(ab*ac, axis=-1), np.sum(ac*ac, axis=-1))
            d20, d21 = np.sum(v*ab, axis=-1), np.sum(v*ac, axis=-1)
            denom = d00*d11-d01*d01
            u, w = (d11*d20-d01*d21)/denom, (d00*d21-d01*d20)/denom
            distance2 = np.where((u >= 0)&(w >= 0)&(u+w <= 1), signed*signed/n2, np.inf)
            for start, end in ((a, b), (b, c), (c, a)):
                edge = end-start
                fraction = np.clip(np.sum((p-start)*edge, axis=-1)/np.sum(edge*edge, axis=-1), 0, 1)
                delta = p-(start+fraction[..., None]*edge)
                distance2 = np.minimum(distance2, np.sum(delta*delta, axis=-1))
            result[rows] = np.any(distance2 <= self.margin**2+1e-15, axis=1)
        return result
