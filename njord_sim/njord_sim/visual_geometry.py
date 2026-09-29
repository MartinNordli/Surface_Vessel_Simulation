"""Export actual SDF visual surfaces, retaining each moving link's TF frame.

Meshes use Gazebo's loader, including its Collada transform handling. Cylinders
are tessellated at 128 sides (maximum radial error <0.00031*r). No convex hull
or bounding envelope is substituted for the visible surface.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import numpy as np

from .constants import BASE_FRAME


def resource_path(uri):
    if uri.startswith('file://'):
        uri = uri[7:]
    if uri.startswith(('model://', 'package://')):
        relative = uri.split('://', 1)[1]
        package, _, tail = relative.partition('/')
        from ament_index_python.packages import get_package_share_directory
        candidates = []
        try:
            candidates.append(Path(get_package_share_directory(package)) / tail)
        except LookupError:
            pass
        candidates += [Path(p) / relative for p in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(':') if p]
        for p in candidates:
            if p.is_file():
                return p.resolve()
        raise ValueError(f'missing visual mesh resource: {uri}')
    path = Path(uri)
    if not path.is_absolute() or not path.is_file():
        raise ValueError(f'visual mesh requires a resolved local resource: {uri}')
    return path.resolve()


def pose_matrix(values):
    x, y, z, r, p, a = values
    cr, sr, cp, sp, ca, sa = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(a), math.sin(a)
    m = np.eye(4)
    m[:3, :3] = [[ca*cp, ca*sp*sr-sa*cr, ca*sp*cr+sa*sr],
                 [sa*cp, sa*sp*sr+ca*cr, sa*sp*cr-ca*sr], [-sp, cp*sr, cp*cr]]
    m[:3, 3] = [x, y, z]
    return m


def export_visual_geometry(model, output):
    output = Path(output)
    surfaces, resources = [], {}
    for link in model.findall('link'):
        for visual in link.findall('visual'):
            pose = visual.find('pose')
            if pose is not None and pose.get('relative_to', link.get('name')) != link.get('name'):
                raise ValueError('visual pose relative_to must be its link after SDF conversion')
            transform = pose_matrix([float(x) for x in visual.findtext('pose', '0 0 0 0 0 0').split()])
            geometry = visual.find('geometry')
            if geometry is None or len(geometry) != 1:
                raise ValueError('visual must have exactly one supported geometry')
            shape = geometry[0]
            if shape.tag == 'mesh':
                if shape.find('submesh') is not None:
                    raise ValueError('explicit SDF submesh selection needs an exporter implementation')
                path = resource_path(shape.findtext('uri', ''))
                resources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
                result = json.loads(subprocess.check_output(['njord_mesh_export', str(path)], text=True))
                vertices, faces = result['vertices'], result['faces']
                scale = np.array([float(x) for x in shape.findtext('scale', '1 1 1').split()])
                vertices = np.asarray(vertices) * scale
            elif shape.tag == 'box':
                from .mesh_geometry import geometry_mesh
                vertices, faces = geometry_mesh(dict(type='box', size_m=[float(x) for x in shape.findtext('size').split()], pose=[0]*6))
            elif shape.tag == 'cylinder':
                radius, length, n = float(shape.findtext('radius')), float(shape.findtext('length')), 128
                vertices = [[radius*math.cos(2*math.pi*i/n), radius*math.sin(2*math.pi*i/n), z] for z in (-length/2, length/2) for i in range(n)]
                vertices += [[0, 0, -length/2], [0, 0, length/2]]
                faces = []
                for i in range(n):
                    j = (i+1) % n
                    faces.extend([[i,j,n+j],[i,n+j,n+i],[2*n,j,i],[2*n+1,n+i,n+j]])
            else:
                raise ValueError(f'unsupported visible geometry: {shape.tag}')
            vertices = np.asarray(vertices) @ transform[:3, :3].T + transform[:3, 3]
            surfaces.append(dict(name=visual.get('name'), frame=link.get('name'), vertices=vertices.tolist(), faces=faces))
    if not surfaces:
        raise ValueError('generated model has no visible vessel surfaces')
    artifact = dict(version=1, frame=BASE_FRAME, surfaces=surfaces, resources=resources)
    (output/'self_geometry.json').write_text(json.dumps(artifact, allow_nan=False)+'\n')
    return resources
